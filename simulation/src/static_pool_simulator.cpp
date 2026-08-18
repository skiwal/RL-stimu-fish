#include "inc/static_pool_simulator.h"

#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/FeatherstoneEntity.h>
#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/Servo.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>


// ============================================================================
// PassiveTailSpringActuator
// ============================================================================
//
// IMPORTANT:
//
// Passive joint torque must be applied during Stonefish's actuator-update
// stage, BEFORE physics integration.
//
// Therefore:
//
//     tau = -k*q - c*qDot
//
// is implemented here instead of SimulationStepCompleted().
//
// ============================================================================

namespace
{

class PassiveTailSpringActuator final
    : public sf::Actuator
{
public:

    PassiveTailSpringActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,
        const std::array<unsigned int, 4>& jointIndices,
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
            Stonefish has no special spring actuator enum.

            We use MOTOR because this custom actuator generates
            joint torque during Actuator::Update().
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
                positionJointType;


            btMultibodyLink::eFeatherstoneJointType
                velocityJointType;


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


            (void)positionJointType;
            (void)velocityJointType;


            // ====================================================
            // Do nothing with an invalid state.
            // ====================================================

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
            // Passive torsional spring-damper
            //
            //     tau = -k*q - c*qDot
            // ====================================================

            sf::Scalar torque =
                -stiffness_
                    * position

                -damping_
                    * velocity;


            // ====================================================
            // Numerical protection
            // ====================================================

            torque =
                std::max(
                    -maxAbsTorque_,
                    std::min(
                        maxAbsTorque_,
                        torque));


            // ====================================================
            // Real Featherstone joint torque
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


    std::array<unsigned int, 4>
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
// Mode names
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
        SimulationStepCompleted() is now ONLY used to:

            record measurements
            print telemetry

        It does NOT apply passive spring torque.
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
    // Bind real fish.
    // ========================================================================

    BindBionicFish();


    // ========================================================================
    // Stage 4C always runs as a decay experiment.
    //
    // Even if somebody accidentally uses "straight", M1 will NOT be given
    // the sinusoidal trajectory from Stage 4B.
    // ========================================================================

    if (
        mode_
        != PoolTestMode::Neutral)
    {
        std::cout
            << "\n"
            << "WARNING:\n"
            << "  Stage 4C ignores active test mode '"
            << PoolTestModeName(
                   mode_)
            << "'.\n"
            << "  Running passive free-decay calibration instead.\n"
            << std::endl;
    }


    // ========================================================================
    // M1 becomes a stationary root clamp.
    // ========================================================================

    ConfigureTailRootHold();


    // ========================================================================
    // Open CSV before applying the initial condition.
    // ========================================================================

    OpenDecayCsv();


    // ========================================================================
    // Give J1~J4 an initial elastic deflection.
    // ========================================================================

    ApplyInitialTailDeflection();


    // ========================================================================
    // Register the actual passive spring actuator.
    //
    // It will start acting when simulation physics begins.
    // ========================================================================

    RegisterPassiveTailSpringActuator();


    // ========================================================================
    // Camera
    // ========================================================================

    ConfigureCamera();


    // ========================================================================
    // Record t=0 state.
    // ========================================================================

    RecordDecaySample();


    PrintDecayTelemetry();


    // ========================================================================
    // Summary
    // ========================================================================

    std::cout
        << "\n"
        << "================================================================\n"
        << " BionicFish V1 - Stage 4C Passive Tail Free Decay\n"
        << "================================================================\n"
        << "\n"
        << "Robot base:\n"
        << "  fixed                         YES\n"
        << "\n"
        << "TailJoint0 / M1:\n"
        << "  periodic drive                NO\n"
        << "  desired position              0 deg\n"
        << "  root holding torque           "
        << rootHoldTorqueNm_
        << " Nm\n"
        << "\n"
        << "Passive joints:\n"
        << "  TailJoint1                    "
        << passiveTailJointIndices_[0]
        << "\n"
        << "  TailJoint2                    "
        << passiveTailJointIndices_[1]
        << "\n"
        << "  TailJoint3                    "
        << passiveTailJointIndices_[2]
        << "\n"
        << "  TailJoint4                    "
        << passiveTailJointIndices_[3]
        << "\n"
        << "\n"
        << "Passive parameters:\n"
        << "  k                             "
        << passiveTailStiffness_
        << " Nm/rad\n"
        << "  c                             "
        << passiveTailDamping_
        << " Nms/rad\n"
        << "  torque clamp                  +/-"
        << passiveTailMaxTorqueNm_
        << " Nm\n"
        << "\n"
        << "Initial deflection:\n"
        << "  J1                            "
        << initialTailDeflectionDeg_[0]
        << " deg\n"
        << "  J2                            "
        << initialTailDeflectionDeg_[1]
        << " deg\n"
        << "  J3                            "
        << initialTailDeflectionDeg_[2]
        << " deg\n"
        << "  J4                            "
        << initialTailDeflectionDeg_[3]
        << " deg\n"
        << "\n"
        << "Initial velocity:\n"
        << "  all passive joints            0 rad/s\n"
        << "\n"
        << "CSV:\n"
        << "  file                          "
        << "tail_decay_stage4c.csv\n"
        << "  sample rate                   100 Hz\n"
        << "\n"
        << "Suggested recording duration:\n"
        << "  "
        << decayMeasurementDuration_
        << " s\n"
        << "\n"
        << "No body-level fake force is used.\n"
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
        fishDynamics_
        == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "BionicFish dynamics are unavailable.");
    }


    fishBody_ =
        fishRobot_
            ->getBaseLink();


    if (
        fishBody_
        == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "BionicFish Body was not found.");
    }


    // ========================================================================
    // TailMotor
    // ========================================================================

    sf::Actuator* actuator =
        fishRobot_
            ->getActuator(
                "BionicFish/TailMotor");


    if (
        actuator == nullptr)
    {
        actuator =
            fishRobot_
                ->getActuator(
                    "TailMotor");
    }


    if (
        actuator == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "TailMotor actuator was not found.");
    }


    tailMotor_ =
        dynamic_cast<sf::Servo*>(
            actuator);


    if (
        tailMotor_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "TailMotor is not sf::Servo.");
    }


    // ========================================================================
    // Passive joints
    // ========================================================================

    passiveTailJointIndices_[0] =
        FindDynamicsJointIndex(
            "TailJoint1");


    passiveTailJointIndices_[1] =
        FindDynamicsJointIndex(
            "TailJoint2");


    passiveTailJointIndices_[2] =
        FindDynamicsJointIndex(
            "TailJoint3");


    passiveTailJointIndices_[3] =
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
                      i + 1)
                + ".");
        }
    }


    std::cout
        << "\n"
        << "BionicFish binding complete:\n"
        << "  Robot       = OK\n"
        << "  Body        = OK\n"
        << "  Dynamics    = OK\n"
        << "  TailMotor   = OK\n"
        << "  TailJoint1  = "
        << passiveTailJointIndices_[0]
        << "\n"
        << "  TailJoint2  = "
        << passiveTailJointIndices_[1]
        << "\n"
        << "  TailJoint3  = "
        << passiveTailJointIndices_[2]
        << "\n"
        << "  TailJoint4  = "
        << passiveTailJointIndices_[3]
        << "\n"
        << std::endl;
}


// ============================================================================
// Find joint
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
        << "Unable to find joint:\n"
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
// M1 root clamp
// ============================================================================

void StaticPoolSimulator::ConfigureTailRootHold()
{
    if (
        tailMotor_ == nullptr)
    {
        throw std::runtime_error(
            "Cannot configure root hold: TailMotor unavailable.");
    }


    tailMotor_
        ->setControlMode(
            sf::ServoControlMode::POSITION);


    tailMotor_
        ->setDesiredPosition(
            0.0);


    tailMotor_
        ->setMaxTorque(
            rootHoldTorqueNm_);


    tailMotor_
        ->setMaxVelocity(
            rootHoldMaxVelocityRadS_);


    std::cout
        << "Tail root clamp configured:\n"
        << "  desired q0    = 0 deg\n"
        << "  max torque    = "
        << rootHoldTorqueNm_
        << " Nm\n"
        << "  max velocity  = "
        << rootHoldMaxVelocityRadS_
        << " rad/s\n"
        << std::endl;
}


// ============================================================================
// Initial tail deflection
// ============================================================================

void StaticPoolSimulator::ApplyInitialTailDeflection()
{
    if (
        fishDynamics_ == nullptr)
    {
        return;
    }


    constexpr sf::Scalar pi =
        3.14159265358979323846;


    constexpr sf::Scalar degToRad =
        pi
        / 180.0;


    std::cout
        << "Applying Stage-4C initial deflection:\n";


    for (
        std::size_t i = 0;
        i < passiveTailJointIndices_.size();
        ++i)
    {
        const unsigned int jointIndex =
            static_cast<unsigned int>(
                passiveTailJointIndices_[i]);


        const sf::Scalar position =
            initialTailDeflectionDeg_[i]
            * degToRad;


        const sf::Scalar velocity =
            0.0;


        fishDynamics_
            ->setJointIC(
                jointIndex,
                position,
                velocity);


        std::cout
            << "  J"
            << (
                   i + 1)
            << " = "
            << initialTailDeflectionDeg_[i]
            << " deg"
            << "  qDot = 0 rad/s\n";
    }


    std::cout
        << std::endl;
}


// ============================================================================
// Register passive tail spring
// ============================================================================

void StaticPoolSimulator::RegisterPassiveTailSpringActuator()
{
    if (
        fishDynamics_ == nullptr)
    {
        throw std::runtime_error(
            "Cannot register passive tail actuator: "
            "dynamics unavailable.");
    }


    const std::array<unsigned int, 4>
        indices =
        {
            static_cast<unsigned int>(
                passiveTailJointIndices_[0]),

            static_cast<unsigned int>(
                passiveTailJointIndices_[1]),

            static_cast<unsigned int>(
                passiveTailJointIndices_[2]),

            static_cast<unsigned int>(
                passiveTailJointIndices_[3])
        };


    PassiveTailSpringActuator* passiveActuator =
        new PassiveTailSpringActuator(
            "BionicFish/PassiveTailSpring",
            fishDynamics_,
            indices,
            passiveTailStiffness_,
            passiveTailDamping_,
            passiveTailMaxTorqueNm_);


    AddActuator(
        passiveActuator);


    std::cout
        << "Passive tail actuator registered:\n"
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
}


// ============================================================================
// Open CSV
// ============================================================================

void StaticPoolSimulator::OpenDecayCsv()
{
    decayCsv_
        .open(
            "tail_decay_stage4c.csv",
            std::ios::out
            |
            std::ios::trunc);


    if (
        !decayCsv_.is_open())
    {
        throw std::runtime_error(
            "Failed to create "
            "tail_decay_stage4c.csv");
    }


    decayCsv_
        << "time_s,"
        << "root_q_rad,"
        << "root_q_deg,"
        << "j1_q_rad,"
        << "j2_q_rad,"
        << "j3_q_rad,"
        << "j4_q_rad,"
        << "j1_q_deg,"
        << "j2_q_deg,"
        << "j3_q_deg,"
        << "j4_q_deg,"
        << "tip_angle_rad,"
        << "tip_angle_deg,"
        << "j1_vel_rad_s,"
        << "j2_vel_rad_s,"
        << "j3_vel_rad_s,"
        << "j4_vel_rad_s,"
        << "j1_tau_nm,"
        << "j2_tau_nm,"
        << "j3_tau_nm,"
        << "j4_tau_nm"
        << "\n";


    decayCsv_
        << std::setprecision(
            10);


    std::cout
        << "Decay CSV opened:\n"
        << "  tail_decay_stage4c.csv\n"
        << std::endl;
}


// ============================================================================
// Read passive tail
// ============================================================================

void StaticPoolSimulator::ReadPassiveTailState(
    std::array<sf::Scalar, 4>& position,
    std::array<sf::Scalar, 4>& velocity,
    std::array<sf::Scalar, 4>& torque) const
{
    position.fill(
        0.0);


    velocity.fill(
        0.0);


    torque.fill(
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
        const unsigned int jointIndex =
            static_cast<unsigned int>(
                passiveTailJointIndices_[i]);


        btMultibodyLink::eFeatherstoneJointType
            positionJointType;


        btMultibodyLink::eFeatherstoneJointType
            velocityJointType;


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


        (void)positionJointType;
        (void)velocityJointType;


        torque[i] =
            -passiveTailStiffness_
                * position[i]

            -passiveTailDamping_
                * velocity[i];


        torque[i] =
            std::max(
                -passiveTailMaxTorqueNm_,
                std::min(
                    passiveTailMaxTorqueNm_,
                    torque[i]));
    }
}


// ============================================================================
// CSV sample
// ============================================================================

void StaticPoolSimulator::RecordDecaySample()
{
    if (
        !decayCsv_.is_open()
        ||
        tailMotor_ == nullptr)
    {
        return;
    }


    std::array<sf::Scalar, 4>
        q;


    std::array<sf::Scalar, 4>
        qDot;


    std::array<sf::Scalar, 4>
        tau;


    ReadPassiveTailState(
        q,
        qDot,
        tau);


    constexpr sf::Scalar radToDeg =
        180.0
        / 3.14159265358979323846;


    const sf::Scalar rootQ =
        tailMotor_
            ->getPosition();


    const sf::Scalar tipAngle =
        q[0]
        + q[1]
        + q[2]
        + q[3];


    decayCsv_
        << elapsedTime_
        << ","

        << rootQ
        << ","

        << rootQ
               * radToDeg
        << ","

        << q[0]
        << ","

        << q[1]
        << ","

        << q[2]
        << ","

        << q[3]
        << ","

        << q[0]
               * radToDeg
        << ","

        << q[1]
               * radToDeg
        << ","

        << q[2]
               * radToDeg
        << ","

        << q[3]
               * radToDeg
        << ","

        << tipAngle
        << ","

        << tipAngle
               * radToDeg
        << ","

        << qDot[0]
        << ","

        << qDot[1]
        << ","

        << qDot[2]
        << ","

        << qDot[3]
        << ","

        << tau[0]
        << ","

        << tau[1]
        << ","

        << tau[2]
        << ","

        << tau[3]

        << "\n";


    ++csvSampleCount_;


    /*
        Flush roughly every 0.5 second.

        This way the data is unlikely to be lost if the GUI is
        closed shortly after the experiment.
    */
    if (
        csvSampleCount_
        % 50
        == 0)
    {
        decayCsv_
            .flush();
    }
}


// ============================================================================
// Console telemetry
// ============================================================================

void StaticPoolSimulator::PrintDecayTelemetry()
{
    if (
        tailMotor_ == nullptr)
    {
        return;
    }


    std::array<sf::Scalar, 4>
        q;


    std::array<sf::Scalar, 4>
        qDot;


    std::array<sf::Scalar, 4>
        tau;


    ReadPassiveTailState(
        q,
        qDot,
        tau);


    constexpr sf::Scalar radToDeg =
        180.0
        / 3.14159265358979323846;


    const sf::Scalar rootQ =
        tailMotor_
            ->getPosition();


    const sf::Scalar tipAngle =
        q[0]
        + q[1]
        + q[2]
        + q[3];


    std::cout
        << std::fixed
        << std::setprecision(
            3)

        << "[Decay] "

        << "t="
        << elapsedTime_

        << " | root="
        << rootQ
               * radToDeg
        << "deg"

        << " | J=("
        << q[0]
               * radToDeg
        << ","
        << q[1]
               * radToDeg
        << ","
        << q[2]
               * radToDeg
        << ","
        << q[3]
               * radToDeg
        << ")deg"

        << " | tip="
        << tipAngle
               * radToDeg
        << "deg"

        << " | qDot=("
        << qDot[0]
        << ","
        << qDot[1]
        << ","
        << qDot[2]
        << ","
        << qDot[3]
        << ")"

        << std::endl;
}


// ============================================================================
// Simulation callback
// ============================================================================

void StaticPoolSimulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTime_ +=
        timeStep;


    // ========================================================================
    // IMPORTANT:
    //
    // No DriveJoint() here.
    //
    // Passive force is generated by PassiveTailSpringActuator::Update().
    // ========================================================================


    // ========================================================================
    // CSV @ 100 Hz
    // ========================================================================

    if (
        lastCsvTime_ < 0.0
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
        lastConsoleTime_ < 0.0
        ||
        (
            elapsedTime_
            - lastConsoleTime_
        )
        >= consolePeriod_)
    {
        PrintDecayTelemetry();


        lastConsoleTime_ =
            elapsedTime_;
    }


    // ========================================================================
    // Tell user when enough data has been recorded.
    // ========================================================================

    if (
        !decayCompletionAnnounced_
        &&
        elapsedTime_
        >= decayMeasurementDuration_)
    {
        decayCompletionAnnounced_ =
            true;


        if (
            decayCsv_.is_open())
        {
            decayCsv_
                .flush();
        }


        std::cout
            << "\n"
            << "============================================================\n"
            << " Stage 4C decay measurement window complete\n"
            << "============================================================\n"
            << "\n"
            << "Recorded approximately "
            << decayMeasurementDuration_
            << " seconds.\n"
            << "\n"
            << "CSV:\n"
            << "  tail_decay_stage4c.csv\n"
            << "\n"
            << "You can close the simulator now.\n"
            << "============================================================\n"
            << std::endl;
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
