#include "inc/static_pool_simulator.h"

#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/FeatherstoneEntity.h>
#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/actuators/Actuator.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>


// ============================================================================
// PassiveFiveJointTailActuator
// ============================================================================
//
// Stage R1:
//
//     TailJoint0
//     TailJoint1
//     TailJoint2
//     TailJoint3
//     TailJoint4
//
// are ALL passive.
//
// This is the discrete surrogate for the continuously flexible
// Nitinol spine.
//
// IMPORTANT:
//
// The torque must be applied from Actuator::Update().
//
// Applying DriveJoint() from SimulationStepCompleted() is incorrect,
// because Stonefish clears multibody forces before actuator updates.
//
// ============================================================================

namespace
{

class PassiveFiveJointTailActuator final
    : public sf::Actuator
{
public:

    PassiveFiveJointTailActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,
        const std::array<unsigned int, 5>& jointIndices,
        sf::Scalar stiffness,
        sf::Scalar damping,
        sf::Scalar maxAbsTorque)
        : sf::Actuator(
              name),
          dynamics_(
              dynamics),
          jointIndices_(
              jointIndices),
          stiffness_(
              stiffness),
          damping_(
              damping),
          maxAbsTorque_(
              std::abs(
                  maxAbsTorque))
    {
    }


    sf::ActuatorType
    getType() const override
    {
        /*
            Stonefish does not provide a dedicated passive torsional
            spring actuator type.

            MOTOR is only used as the generic actuator category here.

            This class does NOT represent M1.
        */
        return
            sf::ActuatorType::MOTOR;
    }


    void Update(
        sf::Scalar timeStep) override
    {
        sf::Actuator::Update(
            timeStep);


        if (
            dynamics_ == nullptr)
        {
            return;
        }


        for (
            unsigned int jointIndex
            : jointIndices_)
        {
            sf::Scalar position =
                0.0;


            sf::Scalar velocity =
                0.0;


            btMultibodyLink::eFeatherstoneJointType
                positionJointType =
                btMultibodyLink::eInvalid;


            btMultibodyLink::eFeatherstoneJointType
                velocityJointType =
                btMultibodyLink::eInvalid;


            dynamics_
                ->getJointPosition(
                    jointIndex,
                    position,
                    positionJointType);


            dynamics_
                ->getJointVelocity(
                    jointIndex,
                    velocity,
                    velocityJointType);


            if (
                positionJointType
                    != btMultibodyLink::eRevolute
                ||
                velocityJointType
                    != btMultibodyLink::eRevolute)
            {
                continue;
            }


            if (
                !std::isfinite(
                    static_cast<double>(
                        position))
                ||
                !std::isfinite(
                    static_cast<double>(
                        velocity)))
            {
                continue;
            }


            // ====================================================
            // Passive flexible-spine joint torque
            //
            //     tau = -k*q - c*qDot
            // ====================================================

            sf::Scalar torque =
                -stiffness_
                    * position

                -damping_
                    * velocity;


            // ====================================================
            // Numerical emergency clamp.
            //
            // It should remain inactive during this experiment.
            // ====================================================

            torque =
                std::max(
                    -maxAbsTorque_,
                    std::min(
                        maxAbsTorque_,
                        torque));


            // ====================================================
            // Internal generalized joint torque.
            //
            // No body-level force is applied.
            // ====================================================

            dynamics_
                ->DriveJoint(
                    jointIndex,
                    torque);
        }
    }


private:

    sf::FeatherstoneEntity* dynamics_ =
        nullptr;


    std::array<unsigned int, 5>
        jointIndices_;


    sf::Scalar stiffness_ =
        0.0;


    sf::Scalar damping_ =
        0.0;


    sf::Scalar maxAbsTorque_ =
        0.0;
};

} // namespace


// ============================================================================
// PoolTestMode name
// ============================================================================

const char*
PoolTestModeName(
    PoolTestMode mode)
{
    switch (mode)
    {
    case PoolTestMode::Neutral:
        return "neutral";


    case PoolTestMode::Straight:
        return "straight";


    case PoolTestMode::TurnLeft:
        return "turn_left";


    case PoolTestMode::TurnRight:
        return "turn_right";


    case PoolTestMode::Dive:
        return "dive";


    case PoolTestMode::Rise:
        return "rise";


    case PoolTestMode::RollLeft:
        return "roll_left";


    case PoolTestMode::RollRight:
        return "roll_right";


    case PoolTestMode::External:
        return "external";
    }


    return "unknown";
}


// ============================================================================
// Constructor
// ============================================================================

StaticPoolSimulator::StaticPoolSimulator(
    sf::Scalar stepsPerSecond,
    PoolTestMode mode)
    : sf::SimulationManager(
          stepsPerSecond),
      mode_(
          mode)
{
    /*
        Stage R1 needs the post-step callback only for telemetry.

        All physical spring torques are applied earlier from
        PassiveFiveJointTailActuator::Update().
    */

    setCallSimulationStepCompleted(
        true);
}


// ============================================================================
// BuildScenario
// ============================================================================

void StaticPoolSimulator::BuildScenario()
{
    sf::SimulationApp* app =
        sf::SimulationApp::getApp();


    if (
        app == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "SimulationApp is not available.");
    }


    const std::string scenarioPath =
        app->getDataPath()
        + "env/static_pool.scn";


    std::cout
        << "\n"
        << "Loading scenario:\n"
        << "  "
        << scenarioPath
        << "\n"
        << std::endl;


    sf::ScenarioParser parser(
        this);


    if (
        !parser.Parse(
            scenarioPath))
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "failed to parse scenario: "
            + scenarioPath);
    }


    // ========================================================================
    // Bind Featherstone fish and ALL five tail joints.
    // ========================================================================

    BindBionicFish();


    // ========================================================================
    // Register passive spine actuator.
    //
    // TailJoint0 is included.
    // ========================================================================

    RegisterPassiveTailSpringActuator();


    // ========================================================================
    // CSV
    // ========================================================================

    OpenDecayCsv();


    // ========================================================================
    // Initial free-decay shape.
    //
    // No motor is used.
    // ========================================================================

    ApplyInitialTailDeflection();


    // ========================================================================
    // Exact t=0 joint-state sample.
    //
    // We only read joint position/velocity here.
    // No Servo solver state is accessed.
    // ========================================================================

    elapsedTime_ =
        0.0;


    RecordDecaySample();


    lastCsvTime_ =
        0.0;


    PrintTelemetry();


    lastConsoleTime_ =
        0.0;


    // ========================================================================
    // Camera
    // ========================================================================

    ConfigureCamera();


    // ========================================================================
    // Summary
    // ========================================================================

    std::cout
        << "\n"
        << "================================================================\n"
        << " BionicFish - Stage R1-A\n"
        << " Five-Passive-Joint Tail Free Decay\n"
        << "================================================================\n"
        << "\n"
        << "Purpose:\n"
        << "  Correct tail topology before tendon implementation.\n"
        << "\n"
        << "Robot base:\n"
        << "  fixed                         YES\n"
        << "\n"
        << "M1 motor:\n"
        << "  active                        NO\n"
        << "  TailJoint0 motor              NONE\n"
        << "\n"
        << "Tendons:\n"
        << "  installed                     NO\n"
        << "\n"
        << "Passive flexible-spine joints:\n"
        << "  TailJoint0                    "
        << passiveTailJointIndices_[0]
        << "\n"
        << "  TailJoint1                    "
        << passiveTailJointIndices_[1]
        << "\n"
        << "  TailJoint2                    "
        << passiveTailJointIndices_[2]
        << "\n"
        << "  TailJoint3                    "
        << passiveTailJointIndices_[3]
        << "\n"
        << "  TailJoint4                    "
        << passiveTailJointIndices_[4]
        << "\n"
        << "\n"
        << "Baseline passive parameters:\n"
        << "  k                             "
        << passiveTailStiffness_
        << " Nm/rad\n"
        << "  c                             "
        << passiveTailDamping_
        << " Nms/rad\n"
        << "  emergency torque clamp        +/-"
        << passiveTailMaxTorqueNm_
        << " Nm\n"
        << "\n"
        << "Initial deflection:\n"
        << "  J0                            "
        << initialTailDeflectionDeg_[0]
        << " deg\n"
        << "  J1                            "
        << initialTailDeflectionDeg_[1]
        << " deg\n"
        << "  J2                            "
        << initialTailDeflectionDeg_[2]
        << " deg\n"
        << "  J3                            "
        << initialTailDeflectionDeg_[3]
        << " deg\n"
        << "  J4                            "
        << initialTailDeflectionDeg_[4]
        << " deg\n"
        << "\n"
        << "Initial joint velocity:\n"
        << "  all                           0 rad/s\n"
        << "\n"
        << "Physics:\n"
        << "  target                        2000 Hz\n"
        << "\n"
        << "CSV:\n"
        << "  file                          tail_decay_stage_r1.csv\n"
        << "  sample rate                   500 Hz\n"
        << "\n"
        << "IMPORTANT:\n"
        << "  Do NOT tune k to 3.5 Hz yet.\n"
        << "  First verify topology and tail mass/inertia consistency.\n"
        << "================================================================\n"
        << std::endl;
}


// ============================================================================
// Bind BionicFish
// ============================================================================

void StaticPoolSimulator::BindBionicFish()
{
    fishRobot_ =
        getRobot(
            "BionicFish");


    if (
        fishRobot_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "robot 'BionicFish' was not found.");
    }


    fishFeatherstoneRobot_ =
        dynamic_cast<sf::FeatherstoneRobot*>(
            fishRobot_);


    if (
        fishFeatherstoneRobot_
        == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "BionicFish is not a FeatherstoneRobot.");
    }


    fishDynamics_ =
        fishFeatherstoneRobot_
            ->getDynamics();


    if (
        fishDynamics_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "BionicFish Featherstone dynamics are unavailable.");
    }


    fishBody_ =
        fishRobot_
            ->getBaseLink();


    if (
        fishBody_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "BionicFish Body was not found.");
    }


    // ========================================================================
    // Stage R1:
    //
    // Bind ALL FIVE passive tail joints.
    //
    // TailJoint0 is no longer treated as M1.
    // ========================================================================

    passiveTailJointIndices_[0] =
        FindDynamicsJointIndex(
            "TailJoint0");


    passiveTailJointIndices_[1] =
        FindDynamicsJointIndex(
            "TailJoint1");


    passiveTailJointIndices_[2] =
        FindDynamicsJointIndex(
            "TailJoint2");


    passiveTailJointIndices_[3] =
        FindDynamicsJointIndex(
            "TailJoint3");


    passiveTailJointIndices_[4] =
        FindDynamicsJointIndex(
            "TailJoint4");


    for (
        std::size_t i = 0;
        i < passiveTailJointIndices_.size();
        ++i)
    {
        if (
            passiveTailJointIndices_[i]
            < 0)
        {
            throw std::runtime_error(
                "StaticPoolSimulator: "
                "failed to locate TailJoint"
                + std::to_string(
                      i)
                + ".");
        }
    }


    std::cout
        << "\n"
        << "BionicFish Stage-R1 binding complete:\n"
        << "  Robot       = OK\n"
        << "  Body        = OK\n"
        << "  Dynamics    = OK\n"
        << "  TailJoint0  = "
        << passiveTailJointIndices_[0]
        << "\n"
        << "  TailJoint1  = "
        << passiveTailJointIndices_[1]
        << "\n"
        << "  TailJoint2  = "
        << passiveTailJointIndices_[2]
        << "\n"
        << "  TailJoint3  = "
        << passiveTailJointIndices_[3]
        << "\n"
        << "  TailJoint4  = "
        << passiveTailJointIndices_[4]
        << "\n"
        << std::endl;
}


// ============================================================================
// Find Featherstone joint index
// ============================================================================

int StaticPoolSimulator::FindDynamicsJointIndex(
    const char* shortJointName) const
{
    if (
        fishDynamics_ == nullptr
        ||
        shortJointName == nullptr)
    {
        return -1;
    }


    const std::string shortName(
        shortJointName);


    const std::string fullName =
        "BionicFish/"
        + shortName;


    const unsigned int jointCount =
        fishDynamics_
            ->getNumOfJoints();


    for (
        unsigned int i = 0;
        i < jointCount;
        ++i)
    {
        const std::string actualName =
            fishDynamics_
                ->getJointName(
                    i);


        if (
            actualName == shortName
            ||
            actualName == fullName)
        {
            return
                static_cast<int>(
                    i);
        }
    }


    std::cerr
        << "\n"
        << "Unable to locate Featherstone joint:\n"
        << "  "
        << shortName
        << "\n"
        << "\n"
        << "Available joints:\n";


    for (
        unsigned int i = 0;
        i < jointCount;
        ++i)
    {
        std::cerr
            << "  ["
            << i
            << "] "
            << fishDynamics_
                   ->getJointName(
                       i)
            << "\n";
    }


    std::cerr
        << std::endl;


    return -1;
}


// ============================================================================
// Register passive five-joint tail
// ============================================================================

void StaticPoolSimulator::RegisterPassiveTailSpringActuator()
{
    if (
        fishDynamics_ == nullptr)
    {
        throw std::runtime_error(
            "Cannot register Stage-R1 passive tail actuator: "
            "fish dynamics are unavailable.");
    }


    const std::array<unsigned int, 5>
        indices =
        {
            static_cast<unsigned int>(
                passiveTailJointIndices_[0]),

            static_cast<unsigned int>(
                passiveTailJointIndices_[1]),

            static_cast<unsigned int>(
                passiveTailJointIndices_[2]),

            static_cast<unsigned int>(
                passiveTailJointIndices_[3]),

            static_cast<unsigned int>(
                passiveTailJointIndices_[4])
        };


    PassiveFiveJointTailActuator* passiveActuator =
        new PassiveFiveJointTailActuator(
            "BionicFish/PassiveFiveJointSpine",
            fishDynamics_,
            indices,
            passiveTailStiffness_,
            passiveTailDamping_,
            passiveTailMaxTorqueNm_);


    /*
        SimulationManager owns the registered actuator.
    */

    AddActuator(
        passiveActuator);


    std::cout
        << "Stage-R1 passive spine actuator registered:\n"
        << "  joints  = TailJoint0~TailJoint4\n"
        << "  k       = "
        << passiveTailStiffness_
        << " Nm/rad\n"
        << "  c       = "
        << passiveTailDamping_
        << " Nms/rad\n"
        << "  tau max = +/-"
        << passiveTailMaxTorqueNm_
        << " Nm\n"
        << std::endl;
}


// ============================================================================
// Apply initial deflection
// ============================================================================

void StaticPoolSimulator::ApplyInitialTailDeflection()
{
    if (
        fishDynamics_ == nullptr)
    {
        throw std::runtime_error(
            "ApplyInitialTailDeflection: "
            "fish dynamics are unavailable.");
    }


    constexpr sf::Scalar pi =
        3.14159265358979323846;


    constexpr sf::Scalar degToRad =
        pi
        / 180.0;


    std::cout
        << "Applying Stage-R1 five-joint initial deflection:\n";


    for (
        std::size_t i = 0;
        i < passiveTailJointIndices_.size();
        ++i)
    {
        const sf::Scalar q =
            initialTailDeflectionDeg_[i]
            * degToRad;


        const unsigned int jointIndex =
            static_cast<unsigned int>(
                passiveTailJointIndices_[i]);


        /*
            Official Featherstone initial-condition API.

            Position:
                specified bend

            Velocity:
                zero
        */

        fishDynamics_
            ->setJointIC(
                jointIndex,
                q,
                0.0);


        std::cout
            << "  J"
            << i
            << " = "
            << initialTailDeflectionDeg_[i]
            << " deg"
            << "  qDot = 0 rad/s\n";
    }


    std::cout
        << std::endl;
}


// ============================================================================
// Read tail state
// ============================================================================

void StaticPoolSimulator::ReadTailState(
    std::array<sf::Scalar, 5>& position,
    std::array<sf::Scalar, 5>& velocity,
    std::array<sf::Scalar, 5>& rawTorque,
    std::array<sf::Scalar, 5>& appliedTorque) const
{
    position.fill(
        0.0);


    velocity.fill(
        0.0);


    rawTorque.fill(
        0.0);


    appliedTorque.fill(
        0.0);


    if (
        fishDynamics_ == nullptr)
    {
        return;
    }


    for (
        std::size_t i = 0;
        i < passiveTailJointIndices_.size();
        ++i)
    {
        const int rawIndex =
            passiveTailJointIndices_[i];


        if (
            rawIndex < 0)
        {
            continue;
        }


        const unsigned int jointIndex =
            static_cast<unsigned int>(
                rawIndex);


        btMultibodyLink::eFeatherstoneJointType
            positionJointType =
            btMultibodyLink::eInvalid;


        btMultibodyLink::eFeatherstoneJointType
            velocityJointType =
            btMultibodyLink::eInvalid;


        fishDynamics_
            ->getJointPosition(
                jointIndex,
                position[i],
                positionJointType);


        fishDynamics_
            ->getJointVelocity(
                jointIndex,
                velocity[i],
                velocityJointType);


        if (
            positionJointType
                != btMultibodyLink::eRevolute
            ||
            velocityJointType
                != btMultibodyLink::eRevolute)
        {
            position[i] =
                0.0;


            velocity[i] =
                0.0;


            continue;
        }


        rawTorque[i] =
            -passiveTailStiffness_
                * position[i]

            -passiveTailDamping_
                * velocity[i];


        appliedTorque[i] =
            std::max(
                -passiveTailMaxTorqueNm_,
                std::min(
                    passiveTailMaxTorqueNm_,
                    rawTorque[i]));
    }
}


// ============================================================================
// Camera
// ============================================================================

void StaticPoolSimulator::ConfigureCamera()
{
    if (
        fishBody_ == nullptr)
    {
        return;
    }


    sf::OpenGLTrackball* trackball =
        getTrackball();


    if (
        trackball == nullptr)
    {
        return;
    }


    trackball
        ->GlueToMoving(
            fishBody_);


    trackball
        ->UpdateCenterPos();


    trackball
        ->MouseScroll(
            -10.5f);


    trackball
        ->UpdateTransform();


    std::cout
        << "Camera configured:\n"
        << "  target   = BionicFish/Body\n"
        << "  tracking = ON\n"
        << std::endl;
}


// ============================================================================
// Open CSV
// ============================================================================

void StaticPoolSimulator::OpenDecayCsv()
{
    decayCsv_
        .open(
            "tail_decay_stage_r1.csv",
            std::ios::out
            |
            std::ios::trunc);


    if (
        !decayCsv_.is_open())
    {
        throw std::runtime_error(
            "Failed to create "
            "tail_decay_stage_r1.csv");
    }


    decayCsv_
        << "time_s,"

        << "j0_q_rad,"
        << "j1_q_rad,"
        << "j2_q_rad,"
        << "j3_q_rad,"
        << "j4_q_rad,"

        << "j0_q_deg,"
        << "j1_q_deg,"
        << "j2_q_deg,"
        << "j3_q_deg,"
        << "j4_q_deg,"

        << "tip_angle_rad,"
        << "tip_angle_deg,"

        << "j0_vel_rad_s,"
        << "j1_vel_rad_s,"
        << "j2_vel_rad_s,"
        << "j3_vel_rad_s,"
        << "j4_vel_rad_s,"

        << "j0_tau_raw_nm,"
        << "j1_tau_raw_nm,"
        << "j2_tau_raw_nm,"
        << "j3_tau_raw_nm,"
        << "j4_tau_raw_nm,"

        << "j0_tau_applied_nm,"
        << "j1_tau_applied_nm,"
        << "j2_tau_applied_nm,"
        << "j3_tau_applied_nm,"
        << "j4_tau_applied_nm,"

        << "j0_clipped,"
        << "j1_clipped,"
        << "j2_clipped,"
        << "j3_clipped,"
        << "j4_clipped"

        << "\n";


    decayCsv_
        << std::setprecision(
            10);


    std::cout
        << "Stage-R1 CSV opened:\n"
        << "  tail_decay_stage_r1.csv\n"
        << std::endl;
}


// ============================================================================
// Record decay sample
// ============================================================================

void StaticPoolSimulator::RecordDecaySample()
{
    if (
        !decayCsv_.is_open()
        ||
        fishDynamics_ == nullptr)
    {
        return;
    }


    constexpr sf::Scalar pi =
        3.14159265358979323846;


    constexpr sf::Scalar radToDeg =
        180.0
        / pi;


    std::array<sf::Scalar, 5>
        q;


    std::array<sf::Scalar, 5>
        qDot;


    std::array<sf::Scalar, 5>
        rawTorque;


    std::array<sf::Scalar, 5>
        appliedTorque;


    ReadTailState(
        q,
        qDot,
        rawTorque,
        appliedTorque);


    sf::Scalar tipAngle =
        0.0;


    for (
        sf::Scalar value
        : q)
    {
        tipAngle +=
            value;
    }


    std::array<int, 5>
        clipped =
        {
            0,
            0,
            0,
            0,
            0
        };


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        clipped[i] =
            std::abs(
                rawTorque[i])
                >
            (
                passiveTailMaxTorqueNm_
                + 1e-9
            )
                ? 1
                : 0;
    }


    decayCsv_
        << elapsedTime_
        << ","

        << q[0]
        << ","
        << q[1]
        << ","
        << q[2]
        << ","
        << q[3]
        << ","
        << q[4]
        << ","

        << q[0] * radToDeg
        << ","
        << q[1] * radToDeg
        << ","
        << q[2] * radToDeg
        << ","
        << q[3] * radToDeg
        << ","
        << q[4] * radToDeg
        << ","

        << tipAngle
        << ","
        << tipAngle * radToDeg
        << ","

        << qDot[0]
        << ","
        << qDot[1]
        << ","
        << qDot[2]
        << ","
        << qDot[3]
        << ","
        << qDot[4]
        << ","

        << rawTorque[0]
        << ","
        << rawTorque[1]
        << ","
        << rawTorque[2]
        << ","
        << rawTorque[3]
        << ","
        << rawTorque[4]
        << ","

        << appliedTorque[0]
        << ","
        << appliedTorque[1]
        << ","
        << appliedTorque[2]
        << ","
        << appliedTorque[3]
        << ","
        << appliedTorque[4]
        << ","

        << clipped[0]
        << ","
        << clipped[1]
        << ","
        << clipped[2]
        << ","
        << clipped[3]
        << ","
        << clipped[4]

        << "\n";


    ++csvSampleCount_;


    /*
        500 Hz.

        Flush every 250 samples:
            approximately every 0.5 s.
    */

    if (
        csvSampleCount_
            % 250
        == 0)
    {
        decayCsv_
            .flush();
    }
}


// ============================================================================
// Console telemetry
// ============================================================================

void StaticPoolSimulator::PrintTelemetry()
{
    if (
        fishDynamics_ == nullptr)
    {
        return;
    }


    constexpr sf::Scalar pi =
        3.14159265358979323846;


    constexpr sf::Scalar radToDeg =
        180.0
        / pi;


    std::array<sf::Scalar, 5>
        q;


    std::array<sf::Scalar, 5>
        qDot;


    std::array<sf::Scalar, 5>
        rawTorque;


    std::array<sf::Scalar, 5>
        appliedTorque;


    ReadTailState(
        q,
        qDot,
        rawTorque,
        appliedTorque);


    sf::Scalar tipAngle =
        0.0;


    int clippedCount =
        0;


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        tipAngle +=
            q[i];


        if (
            std::abs(
                rawTorque[i])
                >
            (
                passiveTailMaxTorqueNm_
                + 1e-9
            ))
        {
            ++clippedCount;
        }
    }


    std::cout
        << std::fixed
        << std::setprecision(
            4)

        << "[StageR1] "

        << "t="
        << elapsedTime_

        << " | J=("

        << q[0] * radToDeg
        << ","

        << q[1] * radToDeg
        << ","

        << q[2] * radToDeg
        << ","

        << q[3] * radToDeg
        << ","

        << q[4] * radToDeg

        << ")deg"

        << " | tip="
        << tipAngle * radToDeg
        << "deg"

        << " | qDot=("

        << qDot[0]
        << ","

        << qDot[1]
        << ","

        << qDot[2]
        << ","

        << qDot[3]
        << ","

        << qDot[4]

        << ")rad/s"

        << " | clips="
        << clippedCount

        << std::endl;
}


// ============================================================================
// SimulationStepCompleted
// ============================================================================

void StaticPoolSimulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTime_ +=
        timeStep;


    // ========================================================================
    // IMPORTANT:
    //
    // Absolutely no DriveJoint() here.
    //
    // Passive torques are already applied by
    // PassiveFiveJointTailActuator::Update().
    // ========================================================================


    // ========================================================================
    // CSV @ 500 Hz
    // ========================================================================

    if (
        lastCsvTime_
            < 0.0

        ||

        (
            elapsedTime_
            - lastCsvTime_
        )
            >= csvPeriod_)
    {
        RecordDecaySample();


        lastCsvTime_ =
            elapsedTime_;
    }


    // ========================================================================
    // Console @ 20 Hz
    // ========================================================================

    if (
        lastConsoleTime_
            < 0.0

        ||

        (
            elapsedTime_
            - lastConsoleTime_
        )
            >= consolePeriod_)
    {
        PrintTelemetry();


        lastConsoleTime_ =
            elapsedTime_;
    }
}


// ============================================================================
// Getters
// ============================================================================

sf::Robot*
StaticPoolSimulator::GetFishRobot() const
{
    return fishRobot_;
}


sf::SolidEntity*
StaticPoolSimulator::GetFishBody() const
{
    return fishBody_;
}


PoolTestMode
StaticPoolSimulator::GetTestMode() const
{
    return mode_;
}
