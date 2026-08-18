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
// Passive spring forces MUST be applied from Actuator::Update().
//
// Stonefish clears multibody forces before actuator updates during
// each physics tick, so this is the correct place for:
//
//     tau = -k*q - c*qDot
//
// SimulationStepCompleted() is used only for:
//     commands
//     safety
//     logging
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
            Stonefish has no dedicated passive-spring actuator type.

            MOTOR is used only as the generic actuator category.
        */

        return
            sf::ActuatorType::MOTOR;
    }


    void Update(
        sf::Scalar timeStep) override
    {
        /*
            Preserve Stonefish Actuator base bookkeeping.
        */

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
            // Safety clamp
            // ====================================================

            torque =
                std::max(
                    -maxAbsTorque_,
                    std::min(
                        maxAbsTorque_,
                        torque));


            // ====================================================
            // Real Featherstone joint torque.
            //
            // This is NOT a body propulsion force.
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
        Required for Stage-5A command generation and telemetry.
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
    // Bind articulated BionicFish.
    // ========================================================================

    BindBionicFish();


    // ========================================================================
    // Register calibrated passive tail.
    // ========================================================================

    RegisterPassiveTailSpringActuator();


    // ========================================================================
    // Configure M1.
    //
    // neutral:
    //     torque = 0
    //
    // straight:
    //     sine command enabled
    // ========================================================================

    ConfigureTailDrive();


    // ========================================================================
    // Camera.
    //
    // Camera setup has already been proven stable in previous stages.
    // ========================================================================

    ConfigureCamera();


    // ========================================================================
    // Open CSV.
    //
    // IMPORTANT:
    //
    // DO NOT call RecordSwimSample() here.
    //
    // BuildScenario() happens before the first completed physics step.
    // All dynamic-state telemetry starts later from
    // SimulationStepCompleted().
    // ========================================================================

    OpenSwimCsv();


    initialBodyStateCaptured_ =
        false;


    // ========================================================================
    // Startup summary.
    //
    // Do NOT query dynamic body/motor state here.
    // ========================================================================

    std::cout
        << "\n"
        << "================================================================\n"
        << " BionicFish V1 - Stage 5A First Free-Swimming Test\n"
        << "================================================================\n"
        << "\n"
        << "Mode:\n"
        << "  "
        << PoolTestModeName(
               mode_)
        << "\n"
        << "\n"
        << "Robot base:\n"
        << "  fixed                         NO\n"
        << "  free swimming                 YES\n"
        << "\n"
        << "Physics:\n"
        << "  required SPS                  2000 Hz\n"
        << "\n"
        << "Passive tail:\n"
        << "  stiffness k                   "
        << passiveTailStiffness_
        << " Nm/rad\n"
        << "  damping c                     "
        << passiveTailDamping_
        << " Nms/rad\n"
        << "  torque clamp                  +/-"
        << passiveTailMaxTorqueNm_
        << " Nm\n"
        << "\n";


    if (
        mode_
        == PoolTestMode::Straight)
    {
        std::cout
            << "M1 TailMotor:\n"
            << "  enabled                       YES\n"
            << "  control                       POSITION\n"
            << "  waveform                      sine\n"
            << "  amplitude                     +/-5 deg\n"
            << "  frequency                     "
            << tailFrequencyHz_
            << " Hz\n"
            << "  max torque                    "
            << tailMaxTorqueNm_
            << " Nm\n"
            << "  max velocity                  "
            << tailMaxVelocityRadS_
            << " rad/s\n"
            << "  start delay                   "
            << driveStartTime_
            << " s\n"
            << "  ramp duration                 "
            << driveRampTime_
            << " s\n";
    }
    else
    {
        std::cout
            << "M1 TailMotor:\n"
            << "  enabled                       NO\n";
    }


    std::cout
        << "\n"
        << "M2 LeftPectoralMotor:\n"
        << "  enabled                       NO\n"
        << "\n"
        << "M3 RightPectoralMotor:\n"
        << "  enabled                       NO\n"
        << "\n"
        << "Body-level propulsion:\n"
        << "  ApplyForce                    NOT USED\n"
        << "\n"
        << "Telemetry:\n"
        << "  CSV                           free_swim_stage5a.csv\n"
        << "  CSV rate                      200 Hz\n"
        << "  console rate                  10 Hz\n"
        << "\n"
        << "Initial body reference:\n"
        << "  captured after first completed physics step\n"
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
    // M1 TailMotor
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
    // Passive tail joints
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
        << "Unable to locate joint:\n"
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
// Register passive elastic tail
// ============================================================================

void StaticPoolSimulator::RegisterPassiveTailSpringActuator()
{
    if (
        fishDynamics_ == nullptr)
    {
        throw std::runtime_error(
            "Cannot register passive tail actuator: "
            "fish dynamics are unavailable.");
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


    /*
        SimulationManager owns registered actuators.
    */

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
// Configure M1
// ============================================================================

void StaticPoolSimulator::ConfigureTailDrive()
{
    if (
        tailMotor_ == nullptr)
    {
        throw std::runtime_error(
            "ConfigureTailDrive: "
            "TailMotor is unavailable.");
    }


    // ========================================================================
    // Position control.
    // ========================================================================

    tailMotor_
        ->setControlMode(
            sf::ServoControlMode::POSITION);


    tailMotor_
        ->setDesiredPosition(
            0.0);


    tailMotor_
        ->setMaxVelocity(
            tailMaxVelocityRadS_);


    lastTailCommandRad_ =
        0.0;


    motorSafetyTripped_ =
        false;


    // ========================================================================
    // Straight:
    //
    // enable real M1 torque.
    // ========================================================================

    if (
        mode_
        == PoolTestMode::Straight)
    {
        tailMotor_
            ->setMaxTorque(
                tailMaxTorqueNm_);


        std::cout
            << "M1 Stage-5A free-swim drive ENABLED.\n"
            << std::endl;
    }
    else
    {
        /*
            Neutral and all currently unsupported modes:
            motor completely off.
        */

        tailMotor_
            ->setMaxTorque(
                0.0);


        tailMotor_
            ->setDesiredPosition(
                0.0);


        std::cout
            << "M1 disabled for mode: "
            << PoolTestModeName(
                   mode_)
            << "\n"
            << std::endl;
    }
}


// ============================================================================
// Update M1 sinusoidal command
// ============================================================================

void StaticPoolSimulator::UpdateTailDriveCommand()
{
    if (
        tailMotor_ == nullptr)
    {
        return;
    }


    // ========================================================================
    // Safety latch or non-straight mode.
    // ========================================================================

    if (
        motorSafetyTripped_
        ||
        mode_
        != PoolTestMode::Straight)
    {
        lastTailCommandRad_ =
            0.0;


        tailMotor_
            ->setDesiredPosition(
                0.0);


        return;
    }


    // ========================================================================
    // First second:
    //
    // fish settles freely with M1 at zero.
    // ========================================================================

    if (
        elapsedTime_
        < driveStartTime_)
    {
        lastTailCommandRad_ =
            0.0;


        tailMotor_
            ->setDesiredPosition(
                0.0);


        return;
    }


    const sf::Scalar driveTime =
        elapsedTime_
        - driveStartTime_;


    // ========================================================================
    // Linear amplitude ramp.
    // ========================================================================

    sf::Scalar ramp =
        1.0;


    if (
        driveRampTime_
        > 0.0)
    {
        ramp =
            driveTime
            / driveRampTime_;
    }


    ramp =
        std::max(
            sf::Scalar(0.0),
            std::min(
                sf::Scalar(1.0),
                ramp));


    constexpr sf::Scalar pi =
        3.14159265358979323846;


    const sf::Scalar omega =
        2.0
        * pi
        * tailFrequencyHz_;


    const sf::Scalar phase =
        omega
        * driveTime;


    lastTailCommandRad_ =
        ramp
        * tailAmplitudeRad_
        * std::sin(
            phase);


    /*
        Persistent Servo setpoint.

        No joint teleportation.
        No Body force.
    */

    tailMotor_
        ->setDesiredPosition(
            lastTailCommandRad_);
}


// ============================================================================
// Read passive tail
// ============================================================================

void StaticPoolSimulator::ReadPassiveTailState(
    std::array<sf::Scalar, 4>& position,
    std::array<sf::Scalar, 4>& velocity,
    std::array<sf::Scalar, 4>& rawTorque,
    std::array<sf::Scalar, 4>& appliedTorque) const
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
// Safety
// ============================================================================

void StaticPoolSimulator::CheckSafety()
{
    /*
        Neutral mode has no active M1 propulsion.

        Stage-5A active safety is needed only during straight mode.
    */

    if (
        motorSafetyTripped_
        ||
        mode_
            != PoolTestMode::Straight
        ||
        tailMotor_
            == nullptr
        ||
        fishBody_
            == nullptr)
    {
        return;
    }


    // ========================================================================
    // M1
    // ========================================================================

    const sf::Scalar m1Position =
        tailMotor_
            ->getPosition();


    const sf::Scalar m1Velocity =
        tailMotor_
            ->getVelocity();


    const sf::Scalar m1Error =
        lastTailCommandRad_
        - m1Position;


    if (
        !std::isfinite(
            static_cast<double>(
                m1Position))
        ||
        !std::isfinite(
            static_cast<double>(
                m1Velocity))
        ||
        !std::isfinite(
            static_cast<double>(
                m1Error)))
    {
        EmergencyDisableTailMotor(
            "non-finite M1 state");

        return;
    }


    if (
        std::abs(
            m1Position)
        > safetyMaxM1PositionRad_)
    {
        EmergencyDisableTailMotor(
            "M1 position exceeded 25 deg");

        return;
    }


    if (
        std::abs(
            m1Velocity)
        > safetyMaxM1VelocityRadS_)
    {
        EmergencyDisableTailMotor(
            "M1 velocity exceeded 2 rad/s");

        return;
    }


    if (
        elapsedTime_
            >
        driveStartTime_
            + 0.25
        &&
        std::abs(
            m1Error)
            >
        safetyMaxM1TrackingErrorRad_)
    {
        EmergencyDisableTailMotor(
            "M1 tracking error exceeded 20 deg");

        return;
    }


    // ========================================================================
    // Passive joints
    // ========================================================================

    std::array<sf::Scalar, 4>
        q;


    std::array<sf::Scalar, 4>
        qDot;


    std::array<sf::Scalar, 4>
        rawTorque;


    std::array<sf::Scalar, 4>
        appliedTorque;


    ReadPassiveTailState(
        q,
        qDot,
        rawTorque,
        appliedTorque);


    for (
        std::size_t i = 0;
        i < 4;
        ++i)
    {
        if (
            !std::isfinite(
                static_cast<double>(
                    q[i]))
            ||
            !std::isfinite(
                static_cast<double>(
                    qDot[i])))
        {
            EmergencyDisableTailMotor(
                "non-finite passive-tail state");

            return;
        }


        if (
            std::abs(
                q[i])
            > safetyMaxPassivePositionRad_)
        {
            EmergencyDisableTailMotor(
                "passive joint exceeded 20 deg");

            return;
        }


        if (
            std::abs(
                qDot[i])
            > safetyMaxPassiveVelocityRadS_)
        {
            EmergencyDisableTailMotor(
                "passive joint velocity exceeded 30 rad/s");

            return;
        }
    }


    // ========================================================================
    // Whole-body state
    //
    // Safe because this function is only called from the completed-step
    // callback after the reference state has been captured.
    // ========================================================================

    const sf::Transform bodyTransform =
        fishBody_
            ->getOTransform();


    const sf::Vector3 bodyPosition =
        bodyTransform
            .getOrigin();


    const sf::Vector3 bodyVelocity =
        fishBody_
            ->getLinearVelocity();


    sf::Scalar yaw =
        0.0;


    sf::Scalar pitch =
        0.0;


    sf::Scalar roll =
        0.0;


    bodyTransform
        .getBasis()
        .getEulerYPR(
            yaw,
            pitch,
            roll);


    if (
        !std::isfinite(
            static_cast<double>(
                bodyPosition.x()))
        ||
        !std::isfinite(
            static_cast<double>(
                bodyPosition.y()))
        ||
        !std::isfinite(
            static_cast<double>(
                bodyPosition.z()))
        ||
        !std::isfinite(
            static_cast<double>(
                bodyVelocity.x()))
        ||
        !std::isfinite(
            static_cast<double>(
                bodyVelocity.y()))
        ||
        !std::isfinite(
            static_cast<double>(
                bodyVelocity.z())))
    {
        EmergencyDisableTailMotor(
            "non-finite Body state");

        return;
    }


    if (
        bodyVelocity.length()
        >
        safetyMaxBodySpeedMS_)
    {
        EmergencyDisableTailMotor(
            "Body speed exceeded 2 m/s");

        return;
    }


    if (
        std::abs(
            roll)
            >
        safetyMaxRollPitchRad_
        ||
        std::abs(
            pitch)
            >
        safetyMaxRollPitchRad_)
    {
        EmergencyDisableTailMotor(
            "Body roll/pitch exceeded 60 deg");

        return;
    }


    if (
        bodyPosition.z()
            <
        safetyMinBodyZ_
        ||
        bodyPosition.z()
            >
        safetyMaxBodyZ_)
    {
        EmergencyDisableTailMotor(
            "Body left safe pool depth range");

        return;
    }
}


// ============================================================================
// Emergency M1 shutdown
// ============================================================================

void StaticPoolSimulator::EmergencyDisableTailMotor(
    const char* reason)
{
    if (
        tailMotor_ == nullptr
        ||
        motorSafetyTripped_)
    {
        return;
    }


    motorSafetyTripped_ =
        true;


    lastTailCommandRad_ =
        0.0;


    /*
        Remove active propulsion capability.
    */

    tailMotor_
        ->setMaxTorque(
            0.0);


    tailMotor_
        ->setDesiredPosition(
            0.0);


    std::cerr
        << "\n"
        << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        << " STAGE 5A M1 SAFETY SHUTDOWN\n"
        << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        << "\n"
        << "Reason:\n"
        << "  "
        << (
               reason != nullptr
                   ? reason
                   : "unknown")
        << "\n"
        << "\n"
        << "TailMotor max torque = 0 Nm\n"
        << "Passive tail remains enabled.\n"
        << "Restart simulator before another active test.\n"
        << "\n"
        << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
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


    std::cout
        << "Camera configured:\n"
        << "  target   = BionicFish/Body\n"
        << "  tracking = ON\n"
        << std::endl;
}


// ============================================================================
// Open Stage-5A CSV
// ============================================================================

void StaticPoolSimulator::OpenSwimCsv()
{
    swimCsv_
        .open(
            "free_swim_stage5a.csv",
            std::ios::out
            |
            std::ios::trunc);


    if (
        !swimCsv_.is_open())
    {
        throw std::runtime_error(
            "Failed to create "
            "free_swim_stage5a.csv");
    }


    /*
        IMPORTANT:

        We intentionally do NOT include Servo::getEffort() in the first
        Stage-5A logger.

        Position and velocity are enough to validate initial free swimming,
        and this avoids touching the motor solver impulse during the startup
        transition.
    */

    swimCsv_
        << "time_s,"
        << "body_x_m,"
        << "body_y_m,"
        << "body_z_m,"
        << "body_dx_m,"
        << "body_dy_m,"
        << "body_dz_m,"
        << "body_vx_m_s,"
        << "body_vy_m_s,"
        << "body_vz_m_s,"
        << "body_wx_rad_s,"
        << "body_wy_rad_s,"
        << "body_wz_rad_s,"
        << "roll_rad,"
        << "pitch_rad,"
        << "yaw_rad,"
        << "roll_deg,"
        << "pitch_deg,"
        << "yaw_deg,"
        << "m1_cmd_rad,"
        << "m1_cmd_deg,"
        << "m1_q_rad,"
        << "m1_q_deg,"
        << "m1_vel_rad_s,"
        << "j1_q_rad,"
        << "j2_q_rad,"
        << "j3_q_rad,"
        << "j4_q_rad,"
        << "j1_q_deg,"
        << "j2_q_deg,"
        << "j3_q_deg,"
        << "j4_q_deg,"
        << "j1_vel_rad_s,"
        << "j2_vel_rad_s,"
        << "j3_vel_rad_s,"
        << "j4_vel_rad_s,"
        << "j1_tau_raw_nm,"
        << "j2_tau_raw_nm,"
        << "j3_tau_raw_nm,"
        << "j4_tau_raw_nm,"
        << "j1_tau_applied_nm,"
        << "j2_tau_applied_nm,"
        << "j3_tau_applied_nm,"
        << "j4_tau_applied_nm,"
        << "j1_clipped,"
        << "j2_clipped,"
        << "j3_clipped,"
        << "j4_clipped,"
        << "tail_tip_relative_rad,"
        << "tail_tip_relative_deg,"
        << "safety_tripped"
        << "\n";


    swimCsv_
        << std::setprecision(
            10);


    std::cout
        << "Free-swim CSV opened:\n"
        << "  free_swim_stage5a.csv\n"
        << std::endl;
}


// ============================================================================
// Record completed-step state
// ============================================================================

void StaticPoolSimulator::RecordSwimSample()
{
    /*
        Absolutely no dynamic state is read until the first completed
        physics step has established our reference state.
    */

    if (
        !initialBodyStateCaptured_
        ||
        !swimCsv_.is_open()
        ||
        fishBody_ == nullptr
        ||
        tailMotor_ == nullptr
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


    // ========================================================================
    // Body
    // ========================================================================

    const sf::Transform bodyTransform =
        fishBody_
            ->getOTransform();


    const sf::Vector3 bodyPosition =
        bodyTransform
            .getOrigin();


    const sf::Vector3 bodyVelocity =
        fishBody_
            ->getLinearVelocity();


    const sf::Vector3 bodyAngularVelocity =
        fishBody_
            ->getAngularVelocity();


    const sf::Scalar dx =
        bodyPosition.x()
        - initialBodyX_;


    const sf::Scalar dy =
        bodyPosition.y()
        - initialBodyY_;


    const sf::Scalar dz =
        bodyPosition.z()
        - initialBodyZ_;


    sf::Scalar yaw =
        0.0;


    sf::Scalar pitch =
        0.0;


    sf::Scalar roll =
        0.0;


    bodyTransform
        .getBasis()
        .getEulerYPR(
            yaw,
            pitch,
            roll);


    // ========================================================================
    // M1
    //
    // Do NOT call getEffort() in this first Stage-5A version.
    // ========================================================================

    const sf::Scalar m1Position =
        tailMotor_
            ->getPosition();


    const sf::Scalar m1Velocity =
        tailMotor_
            ->getVelocity();


    // ========================================================================
    // Passive tail
    // ========================================================================

    std::array<sf::Scalar, 4>
        q;


    std::array<sf::Scalar, 4>
        qDot;


    std::array<sf::Scalar, 4>
        rawTorque;


    std::array<sf::Scalar, 4>
        appliedTorque;


    ReadPassiveTailState(
        q,
        qDot,
        rawTorque,
        appliedTorque);


    std::array<int, 4>
        clipped =
        {
            0,
            0,
            0,
            0
        };


    for (
        std::size_t i = 0;
        i < 4;
        ++i)
    {
        /*
            Give a very small floating-point tolerance.
        */

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


    // ========================================================================
    // Relative tail-tip angle
    // ========================================================================

    const sf::Scalar tailTipRelative =
        m1Position

        + q[0]
        + q[1]
        + q[2]
        + q[3];


    // ========================================================================
    // CSV row
    // ========================================================================

    swimCsv_
        << elapsedTime_
        << ","

        << bodyPosition.x()
        << ","
        << bodyPosition.y()
        << ","
        << bodyPosition.z()
        << ","

        << dx
        << ","
        << dy
        << ","
        << dz
        << ","

        << bodyVelocity.x()
        << ","
        << bodyVelocity.y()
        << ","
        << bodyVelocity.z()
        << ","

        << bodyAngularVelocity.x()
        << ","
        << bodyAngularVelocity.y()
        << ","
        << bodyAngularVelocity.z()
        << ","

        << roll
        << ","
        << pitch
        << ","
        << yaw
        << ","

        << roll
               * radToDeg
        << ","
        << pitch
               * radToDeg
        << ","
        << yaw
               * radToDeg
        << ","

        << lastTailCommandRad_
        << ","
        << lastTailCommandRad_
               * radToDeg
        << ","

        << m1Position
        << ","
        << m1Position
               * radToDeg
        << ","

        << m1Velocity
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

        << qDot[0]
        << ","
        << qDot[1]
        << ","
        << qDot[2]
        << ","
        << qDot[3]
        << ","

        << rawTorque[0]
        << ","
        << rawTorque[1]
        << ","
        << rawTorque[2]
        << ","
        << rawTorque[3]
        << ","

        << appliedTorque[0]
        << ","
        << appliedTorque[1]
        << ","
        << appliedTorque[2]
        << ","
        << appliedTorque[3]
        << ","

        << clipped[0]
        << ","
        << clipped[1]
        << ","
        << clipped[2]
        << ","
        << clipped[3]
        << ","

        << tailTipRelative
        << ","
        << tailTipRelative
               * radToDeg
        << ","

        << (
               motorSafetyTripped_
                   ? 1
                   : 0)

        << "\n";


    ++csvSampleCount_;


    /*
        Flush approximately every 0.5 s:

            200 Hz / 100 samples
    */

    if (
        csvSampleCount_
            % 100
        == 0)
    {
        swimCsv_
            .flush();
    }
}


// ============================================================================
// Console telemetry
// ============================================================================

void StaticPoolSimulator::PrintTelemetry()
{
    if (
        !initialBodyStateCaptured_
        ||
        fishBody_ == nullptr
        ||
        tailMotor_ == nullptr)
    {
        return;
    }


    constexpr sf::Scalar pi =
        3.14159265358979323846;


    constexpr sf::Scalar radToDeg =
        180.0
        / pi;


    const sf::Transform bodyTransform =
        fishBody_
            ->getOTransform();


    const sf::Vector3 bodyPosition =
        bodyTransform
            .getOrigin();


    const sf::Vector3 bodyVelocity =
        fishBody_
            ->getLinearVelocity();


    const sf::Scalar dx =
        bodyPosition.x()
        - initialBodyX_;


    const sf::Scalar dy =
        bodyPosition.y()
        - initialBodyY_;


    const sf::Scalar dz =
        bodyPosition.z()
        - initialBodyZ_;


    sf::Scalar yaw =
        0.0;


    sf::Scalar pitch =
        0.0;


    sf::Scalar roll =
        0.0;


    bodyTransform
        .getBasis()
        .getEulerYPR(
            yaw,
            pitch,
            roll);


    std::array<sf::Scalar, 4>
        q;


    std::array<sf::Scalar, 4>
        qDot;


    std::array<sf::Scalar, 4>
        rawTorque;


    std::array<sf::Scalar, 4>
        appliedTorque;


    ReadPassiveTailState(
        q,
        qDot,
        rawTorque,
        appliedTorque);


    int clippedCount =
        0;


    for (
        std::size_t i = 0;
        i < 4;
        ++i)
    {
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


    const sf::Scalar tailTipRelative =
        tailMotor_
            ->getPosition()

        + q[0]
        + q[1]
        + q[2]
        + q[3];


    std::cout
        << std::fixed
        << std::setprecision(
            3)

        << "[Stage5A] "

        << "t="
        << elapsedTime_

        << " | dXYZ=("
        << dx
        << ","
        << dy
        << ","
        << dz
        << ")m"

        << " | vXYZ=("
        << bodyVelocity.x()
        << ","
        << bodyVelocity.y()
        << ","
        << bodyVelocity.z()
        << ")m/s"

        << " | RPY=("
        << roll
               * radToDeg
        << ","
        << pitch
               * radToDeg
        << ","
        << yaw
               * radToDeg
        << ")deg"

        << " | M1cmd="
        << lastTailCommandRad_
               * radToDeg
        << "deg"

        << " M1q="
        << tailMotor_
               ->getPosition()
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
        << tailTipRelative
               * radToDeg
        << "deg"

        << " | clips="
        << clippedCount

        << " | safety="
        << (
               motorSafetyTripped_
                   ? "TRIPPED"
                   : "OK")

        << std::endl;
}


// ============================================================================
// Completed simulation step
// ============================================================================

void StaticPoolSimulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTime_ +=
        timeStep;


    // ========================================================================
    // FIRST COMPLETED PHYSICS STEP
    //
    // Capture our free-swim reference here.
    //
    // This is the major fix for the previous segmentation fault.
    //
    // BuildScenario() no longer reads dynamic body or motor solver state.
    // ========================================================================

    if (
        !initialBodyStateCaptured_)
    {
        if (
            fishBody_ == nullptr)
        {
            return;
        }


        const sf::Transform bodyTransform =
            fishBody_
                ->getOTransform();


        const sf::Vector3 bodyPosition =
            bodyTransform
                .getOrigin();


        initialBodyX_ =
            bodyPosition.x();


        initialBodyY_ =
            bodyPosition.y();


        initialBodyZ_ =
            bodyPosition.z();


        initialBodyStateCaptured_ =
            true;


        lastCsvTime_ =
            -1.0;


        lastConsoleTime_ =
            -1.0;


        std::cout
            << "\n"
            << "Stage-5A free-swim reference captured:\n"
            << "  t = "
            << elapsedTime_
            << " s\n"
            << "  x = "
            << initialBodyX_
            << " m\n"
            << "  y = "
            << initialBodyY_
            << " m\n"
            << "  z = "
            << initialBodyZ_
            << " m\n"
            << std::endl;
    }


    // ========================================================================
    // Update persistent M1 desired position.
    //
    // In neutral mode this remains exactly zero.
    //
    // In straight mode:
    //
    //     0~1 s:
    //         0 deg
    //
    //     1~2 s:
    //         amplitude ramp
    //
    //     >2 s:
    //         +/-5 deg @ 0.4 Hz
    // ========================================================================

    UpdateTailDriveCommand();


    // ========================================================================
    // Active-test safety
    // ========================================================================

    CheckSafety();


    // ========================================================================
    // CSV @ 200 Hz
    // ========================================================================

    if (
        initialBodyStateCaptured_
        &&
        (
            lastCsvTime_
                < 0.0

            ||

            (
                elapsedTime_
                - lastCsvTime_
            )
                >= csvPeriod_
        ))
    {
        RecordSwimSample();


        lastCsvTime_ =
            elapsedTime_;
    }


    // ========================================================================
    // Console @ 10 Hz
    // ========================================================================

    if (
        initialBodyStateCaptured_
        &&
        (
            lastConsoleTime_
                < 0.0

            ||

            (
                elapsedTime_
                - lastConsoleTime_
            )
                >= consolePeriod_
        ))
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
