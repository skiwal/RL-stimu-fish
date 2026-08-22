#include "inc/tendon_stage_r2_simulator.h"

#include <Stonefish/actuators/Servo.h>

#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/FeatherstoneEntity.h>
#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>


namespace
{

constexpr sf::Scalar kRadToDeg =
    57.29577951308232;

constexpr sf::Scalar kTwoPi =
    6.283185307179586;

}



TendonStageR2Simulator::
TendonStageR2Simulator(
    sf::Scalar stepsPerSecond)

    : sf::SimulationManager(
          stepsPerSecond)
{
    setCallSimulationStepCompleted(
        true);
}



void
TendonStageR2Simulator::BuildScenario()
{
    auto* app =
        sf::SimulationApp::getApp();

    if (!app)
        throw std::runtime_error(
            "SimulationApp unavailable.");

    const std::string scenario =
        app->getDataPath()
        +
        "env/static_pool.scn";

    std::cout
        << "\nLoading:\n  "
        << scenario
        << "\n";

    sf::ScenarioParser parser(this);

    if (!parser.Parse(scenario))
    {
        throw std::runtime_error(
            "Failed to parse static_pool.scn");
    }

    BindFish();

    SetNeutralInitialCondition();

    ConfigureMotor();

    RegisterTendonActuator();

    OpenCsv();

    ConfigureCamera();


    std::cout
        << "\n============================================================\n"
        << " STAGE 3 - SOURCE-FAITHFUL DUAL ELASTIC TENDON\n"
        << "============================================================\n"
        << "Body                  FIXED\n"
        << "CaudalFin             FIXED TO Tail4\n"
        << "Fluid                 WET / SUBMERGED\n"
        << "Motor                  REAL M1Joint velocity servo\n"
        << "Motor frequency        "
        << motorFrequencyHz_
        << " Hz\n"
        << "Motor target velocity  "
        << motorTargetVelocityRadS_
        << " rad/s\n"
        << "Motor velocity gain    100\n"
        << "Motor max torque       "
        << motorMaxTorqueNm_
        << " Nm\n"
        << "Tendon stiffness       20000 N/m\n"
        << "Tendon damping         10 Ns/m\n"
        << "Routing                [0,0,0,1,1]\n"
        << "Tail hinge k           0.65 Nm/rad\n"
        << "Tail hinge c           0.0 Nms/rad\n"
        << "CSV                    dual_tendon_motor.csv\n"
        << "============================================================\n\n";
}



void
TendonStageR2Simulator::BindFish()
{
    fishRobot_ =
        getRobot("BionicFish");

    if (!fishRobot_)
    {
        throw std::runtime_error(
            "BionicFish not found.");
    }


    fishFeatherstoneRobot_ =
        dynamic_cast<
            sf::FeatherstoneRobot*>(
                fishRobot_);

    if (!fishFeatherstoneRobot_)
    {
        throw std::runtime_error(
            "BionicFish is not FeatherstoneRobot.");
    }


    fishDynamics_ =
        fishFeatherstoneRobot_
            ->getDynamics();

    fishBody_ =
        fishRobot_
            ->getBaseLink();

    if (!fishDynamics_ ||
        !fishBody_)
    {
        throw std::runtime_error(
            "Fish dynamics unavailable.");
    }


    bodyLinkIndex_ =
        FindLinkIndex("Body");

    motorLinkIndex_ =
        FindLinkIndex("MotorShaft");

    motorJointIndex_ =
        FindJointIndex("M1Joint");


    if (bodyLinkIndex_ != 0 ||
        motorLinkIndex_ < 0 ||
        motorJointIndex_ < 0)
    {
        throw std::runtime_error(
            "Cannot bind Body / MotorShaft / M1Joint.");
    }


    for (std::size_t i=0;
         i<5;
         ++i)
    {
        const std::string jointName =
            "TailJoint"
            +
            std::to_string(i);

        const std::string linkName =
            "Tail"
            +
            std::to_string(i);

        tailJointIndices_[i] =
            FindJointIndex(
                jointName.c_str());

        tailLinkIndices_[i] =
            FindLinkIndex(
                linkName.c_str());

        if (tailJointIndices_[i] < 0 ||
            tailLinkIndices_[i] < 0)
        {
            throw std::runtime_error(
                "Cannot bind "
                + jointName
                + " / "
                + linkName);
        }
    }


    // --------------------------------------------------------
    // ScenarioParser prefixes robot actuator names.
    // Try exact scoped name first, then scan as fallback.
    // --------------------------------------------------------

    auto* actuator =
        fishRobot_
            ->getActuator(
                "BionicFish/M1Servo");

    if (actuator)
    {
        motorServo_ =
            dynamic_cast<sf::Servo*>(
                actuator);
    }


    if (!motorServo_)
    {
        for (std::size_t i=0; ; ++i)
        {
            auto* a =
                fishRobot_
                    ->getActuator(i);

            if (!a)
                break;

            if (a->getName().find(
                    "M1Servo")
                !=
                std::string::npos)
            {
                motorServo_ =
                    dynamic_cast<sf::Servo*>(
                        a);

                if (motorServo_)
                    break;
            }
        }
    }


    if (!motorServo_)
    {
        throw std::runtime_error(
            "M1Servo not found. "
            "Run patch_stage3_motor.py first.");
    }


    std::cout
        << "\nStage 3 binding:\n"
        << "  motor joint = "
        << fishDynamics_
               ->getJointName(
                   static_cast<unsigned int>(
                       motorJointIndex_))
        << "\n"
        << "  motor link  = "
        << fishDynamics_
               ->getLink(
                   static_cast<unsigned int>(
                       motorLinkIndex_))
               .solid
               ->getName()
        << "\n";

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        std::cout
            << "  J"
            << i
            << " = "
            << fishDynamics_
                   ->getJointName(
                       static_cast<unsigned int>(
                           tailJointIndices_[i]))
            << "\n";
    }

    std::cout << std::endl;
}



int
TendonStageR2Simulator::FindJointIndex(
    const char* name) const
{
    if (!fishDynamics_ ||
        !name)
    {
        return -1;
    }

    const std::string plain(name);

    const std::string scoped =
        "BionicFish/"
        +
        plain;

    for (unsigned int i=0;
         i<fishDynamics_->getNumOfJoints();
         ++i)
    {
        const std::string n =
            fishDynamics_
                ->getJointName(i);

        if (n == plain ||
            n == scoped)
        {
            return
                static_cast<int>(i);
        }
    }

    return -1;
}



int
TendonStageR2Simulator::FindLinkIndex(
    const char* name) const
{
    if (!fishDynamics_ ||
        !name)
    {
        return -1;
    }

    const std::string plain(name);

    const std::string scoped =
        "BionicFish/"
        +
        plain;

    for (unsigned int i=0;
         i<fishDynamics_->getNumOfLinks();
         ++i)
    {
        auto* solid =
            fishDynamics_
                ->getLink(i)
                .solid;

        if (!solid)
            continue;

        const std::string n =
            solid->getName();

        if (n == plain ||
            n == scoped)
        {
            return
                static_cast<int>(i);
        }
    }

    return -1;
}



void
TendonStageR2Simulator::
SetNeutralInitialCondition()
{
    fishDynamics_->setJointIC(
        static_cast<unsigned int>(
            motorJointIndex_),
        0.0,
        0.0);

    for (int joint :
         tailJointIndices_)
    {
        fishDynamics_->setJointIC(
            static_cast<unsigned int>(
                joint),
            0.0,
            0.0);
    }
}



void
TendonStageR2Simulator::
ConfigureMotor()
{
    if (!motorServo_)
    {
        throw std::runtime_error(
            "M1Servo unavailable.");
    }

    motorServo_->setControlMode(
        sf::ServoControlMode::VELOCITY);

    motorServo_->setMaxVelocity(
        motorMaxVelocityRadS_);

    motorServo_->setMaxTorque(
        motorMaxTorqueNm_);

    motorServo_->setDesiredVelocity(
        motorTargetVelocityRadS_);


    std::cout
        << "Motor enabled:\n"
        << "  mode       = VELOCITY\n"
        << "  target     = "
        << motorTargetVelocityRadS_
        << " rad/s\n"
        << "  frequency  = "
        << motorFrequencyHz_
        << " Hz\n"
        << "  max torque = "
        << motorMaxTorqueNm_
        << " Nm\n"
        << std::endl;
}



void
TendonStageR2Simulator::
RegisterTendonActuator()
{
    TendonTailParameters p;

    p.passiveStiffnessNmRad =
        0.65;

    p.passiveDampingNmsRad =
        0.0;

    p.tendonStiffnessNPerM =
        20000.0;

    p.tendonDampingNsPerM =
        10.0;

    p.tendonForceSafetyLimitN =
        2000.0;

    p.jointSafetyLimitRad =
        1.3962634016;


    std::array<unsigned int,5>
        joints{};

    std::array<unsigned int,5>
        links{};

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        joints[i] =
            static_cast<unsigned int>(
                tailJointIndices_[i]);

        links[i] =
            static_cast<unsigned int>(
                tailLinkIndices_[i]);
    }


    tendonActuator_ =
        new TendonTailActuator(
            "BionicFish/DualElasticTendon",

            fishDynamics_,

            static_cast<unsigned int>(
                motorLinkIndex_),

            static_cast<unsigned int>(
                motorJointIndex_),

            links,
            joints,
            p);


    AddActuator(
        tendonActuator_);
}



void
TendonStageR2Simulator::
ConfigureCamera()
{
    auto* tb =
        getTrackball();

    if (!tb ||
        !fishBody_)
    {
        return;
    }

    tb->GlueToMoving(
        fishBody_);

    tb->UpdateCenterPos();

    tb->MouseScroll(
        -10.5f);

    tb->UpdateTransform();
}



void
TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "dual_tendon_motor.csv",
        std::ios::out
        |
        std::ios::trunc);

    if (!csv_)
    {
        throw std::runtime_error(
            "Cannot create dual_tendon_motor.csv");
    }


    csv_
        << "time_s,"
        << "motor_angle_rad,"
        << "motor_velocity_rad_s,"
        << "motor_frequency_hz,"
        << "motor_effort_nm,"

        << "left_length_m,"
        << "right_length_m,"

        << "left_rest_length_m,"
        << "right_rest_length_m,"

        << "left_delta_length_m,"
        << "right_delta_length_m,"

        << "left_length_velocity_m_s,"
        << "right_length_velocity_m_s,"

        << "left_force_n,"
        << "right_force_n,";


    for (int i=0;
         i<5;
         ++i)
    {
        csv_
            << "j"
            << i
            << "_deg,";
    }


    for (int i=0;
         i<5;
         ++i)
    {
        csv_
            << "left_tau_j"
            << i
            << "_nm,";
    }


    for (int i=0;
         i<5;
         ++i)
    {
        csv_
            << "right_tau_j"
            << i
            << "_nm,";
    }


    for (int i=0;
         i<5;
         ++i)
    {
        csv_
            << "total_tau_j"
            << i
            << "_nm,";
    }


    csv_
        << "left_anchor_fx_n,"
        << "left_anchor_fy_n,"
        << "left_anchor_fz_n,"

        << "right_anchor_fx_n,"
        << "right_anchor_fy_n,"
        << "right_anchor_fz_n,"

        << "force_residual_n,"
        << "torque_residual_nm,"
        << "safety_tripped\n";


    csv_
        << std::setprecision(12);
}



void
TendonStageR2Simulator::
RecordSample()
{
    if (!csv_ ||
        !tendonActuator_ ||
        !motorServo_)
    {
        return;
    }

    const auto s =
        tendonActuator_
            ->GetSnapshot();


    csv_
        << s.timeS
        << ","
        << s.motorPositionRad
        << ","
        << s.motorVelocityRadS
        << ","
        << s.motorVelocityRadS/kTwoPi
        << ","
        << motorServo_->getEffort()
        << ","

        << s.tendonLengthM[0]
        << ","
        << s.tendonLengthM[1]
        << ","

        << s.restLengthM[0]
        << ","
        << s.restLengthM[1]
        << ","

        << s.tendonDeltaLengthM[0]
        << ","
        << s.tendonDeltaLengthM[1]
        << ","

        << s.tendonLengthVelocityMS[0]
        << ","
        << s.tendonLengthVelocityMS[1]
        << ","

        << s.tendonForceN[0]
        << ","
        << s.tendonForceN[1]
        << ",";


    for (std::size_t i=0;
         i<5;
         ++i)
    {
        csv_
            << s.jointPositionRad[i]
               * kRadToDeg
            << ",";
    }


    for (sf::Scalar v :
         s.tendonTorqueNm[0])
    {
        csv_
            << v
            << ",";
    }


    for (sf::Scalar v :
         s.tendonTorqueNm[1])
    {
        csv_
            << v
            << ",";
    }


    for (sf::Scalar v :
         s.totalTendonTorqueNm)
    {
        csv_
            << v
            << ",";
    }


    csv_
        << s.motorAnchorForceWorld[0].x()
        << ","
        << s.motorAnchorForceWorld[0].y()
        << ","
        << s.motorAnchorForceWorld[0].z()
        << ","

        << s.motorAnchorForceWorld[1].x()
        << ","
        << s.motorAnchorForceWorld[1].y()
        << ","
        << s.motorAnchorForceWorld[1].z()
        << ","

        << s.tendonNetForceResidualWorld.length()
        << ","
        << s.tendonNetTorqueResidualWorld.length()
        << ","

        << (s.safetyTripped ? 1 : 0)
        << "\n";
}



void
TendonStageR2Simulator::
PrintTelemetry()
{
    if (!tendonActuator_ ||
        !motorServo_)
    {
        return;
    }

    const auto s =
        tendonActuator_
            ->GetSnapshot();


    std::cout
        << std::fixed
        << std::setprecision(4)

        << "[MOTOR-TENDON] t="
        << s.timeS

        << " motor=("
        << s.motorPositionRad
        << " rad,"
        << s.motorVelocityRadS/kTwoPi
        << " Hz,"
        << motorServo_->getEffort()
        << " Nm)"

        << " dLmm=("
        << s.tendonDeltaLengthM[0]*1000.0
        << ","
        << s.tendonDeltaLengthM[1]*1000.0
        << ")"

        << " Ldot=("
        << s.tendonLengthVelocityMS[0]
        << ","
        << s.tendonLengthVelocityMS[1]
        << ")"

        << " F=("
        << s.tendonForceN[0]
        << ","
        << s.tendonForceN[1]
        << ")"

        << " Jdeg=(";


    for (std::size_t i=0;
         i<5;
         ++i)
    {
        if (i)
            std::cout << ",";

        std::cout
            << s.jointPositionRad[i]
               * kRadToDeg;
    }


    std::cout
        << ") tau=(";


    for (std::size_t i=0;
         i<5;
         ++i)
    {
        if (i)
            std::cout << ",";

        std::cout
            << s.totalTendonTorqueNm[i];
    }


    std::cout
        << ") residual=("
        << s.tendonNetForceResidualWorld.length()
        << ","
        << s.tendonNetTorqueResidualWorld.length()
        << ")"

        << " safety="
        << (
            s.safetyTripped
            ? "TRIPPED"
            : "OK"
        )

        << std::endl;
}



void
TendonStageR2Simulator::
SimulationStepCompleted(
    sf::Scalar dt)
{
    elapsedTimeS_ += dt;


    if (tendonActuator_)
    {
        const auto s =
            tendonActuator_
                ->GetSnapshot();

        if (s.safetyTripped &&
            motorServo_)
        {
            motorServo_
                ->setDesiredVelocity(
                    0.0);
        }
    }


    if (lastCsvTimeS_ < 0.0 ||
        elapsedTimeS_
            - lastCsvTimeS_
            >= csvPeriodS_)
    {
        RecordSample();

        lastCsvTimeS_ =
            elapsedTimeS_;
    }


    if (lastConsoleTimeS_ < 0.0 ||
        elapsedTimeS_
            - lastConsoleTimeS_
            >= consolePeriodS_)
    {
        PrintTelemetry();

        lastConsoleTimeS_ =
            elapsedTimeS_;
    }
}
