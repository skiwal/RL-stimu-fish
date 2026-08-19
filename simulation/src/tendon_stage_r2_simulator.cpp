#include "inc/tendon_stage_r2_simulator.h"
#include "inc/tendon_tail_actuator.h"

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
    57.2957795130823208768;

}


TendonStageR2Simulator::TendonStageR2Simulator(
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
    sf::SimulationApp*
        app =
            sf::SimulationApp::getApp();


    if (
        app == nullptr)
    {
        throw std::runtime_error(
            "TendonStageR2Simulator: SimulationApp unavailable.");
    }


    const std::string
        scenario =
            app->getDataPath()
            +
            "env/static_pool.scn";


    std::cout
        << "\nLoading:\n  "
        << scenario
        << "\n";


    sf::ScenarioParser
        parser(
            this);


    if (
        !parser.Parse(
            scenario))
    {
        throw std::runtime_error(
            "Failed to parse static_pool.scn");
    }


    BindFish();

    SetNeutralInitialCondition();

    RegisterTendonActuator();

    OpenCsv();

    ConfigureCamera();


    const auto
        snapshot =
            tendonActuator_->GetSnapshot();


    std::cout
        << "\n"
        << "============================================================\n"
        << " BionicFish Stage R2-B\n"
        << " Real M1 + Spatial Pull-Only Tendon Drive\n"
        << "============================================================\n"
        << "Body fixed                 YES\n"
        << "M1                          Stonefish Servo\n"
        << "M1 mode                     velocity\n"
        << "Motor shaft                 real Featherstone link\n"
        << "Crank                       virtual geometry\n"
        << "Tendon                      virtual elastic cable\n"
        << "Tendon can push             NO\n"
        << "Tendon forces               physical link forces\n"
        << "Manual tendon joint torque  NO\n"
        << "Routing                     0,0,0,1,1\n"
        << "Tendon stiffness            20000 N/m\n"
        << "Tendon damping              10 N*s/m\n"
        << "Initial pretension          0 N\n"
        << "Diagnostic strain limit     3 %\n"
        << "Passive joint stiffness     0.65 Nm/rad\n"
        << "Motor diagnostic freq       0.05 Hz\n"
        << "Left free length            "
        << snapshot.tendonFreeLengthM[0]
        << " m\n"
        << "Right free length           "
        << snapshot.tendonFreeLengthM[1]
        << " m\n"
        << "CSV                         tendon_stage_r2b.csv\n"
        << "============================================================\n"
        << std::endl;
}


void
TendonStageR2Simulator::BindFish()
{
    fishRobot_ =
        getRobot(
            "BionicFish");


    if (
        fishRobot_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish not found.");
    }


    fishFeatherstoneRobot_ =
        dynamic_cast<sf::FeatherstoneRobot*>(
            fishRobot_);


    if (
        fishFeatherstoneRobot_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish is not a FeatherstoneRobot.");
    }


    fishDynamics_ =
        fishFeatherstoneRobot_->getDynamics();


    fishBody_ =
        fishRobot_->getBaseLink();


    if (
        fishDynamics_ == nullptr
        ||
        fishBody_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish dynamics/base unavailable.");
    }


    // ============================================================
    // Bind real M1
    // ============================================================

    m1JointIndex_ =
        FindJointIndex(
            "M1Joint");


    if (
        m1JointIndex_ < 0)
    {
        throw std::runtime_error(
            "Cannot find M1Joint.");
    }


    motorShaftLinkIndex_ =
        FindLinkIndex(
            "MotorShaft");


    if (
        motorShaftLinkIndex_ < 0)
    {
        throw std::runtime_error(
            "Cannot find MotorShaft.");
    }


    sf::Actuator*
        rawM1 =
            fishRobot_->getActuator(
                "M1Servo");


    if (
        rawM1 == nullptr)
    {
        rawM1 =
            fishRobot_->getActuator(
                "BionicFish/M1Servo");
    }


    m1Servo_ =
        dynamic_cast<sf::Servo*>(
            rawM1);


    if (
        m1Servo_ == nullptr)
    {
        throw std::runtime_error(
            "Cannot find/convert M1Servo.");
    }


    // ============================================================
    // Bind tail joints and links
    // ============================================================

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        const std::string
            jointName =
                "TailJoint"
                +
                std::to_string(
                    i);


        const std::string
            linkName =
                "Tail"
                +
                std::to_string(
                    i);


        tailJointIndices_[i] =
            FindJointIndex(
                jointName.c_str());


        tailLinkIndices_[i] =
            FindLinkIndex(
                linkName.c_str());


        if (
            tailJointIndices_[i] < 0)
        {
            throw std::runtime_error(
                "Cannot find "
                +
                jointName);
        }


        if (
            tailLinkIndices_[i] < 0)
        {
            throw std::runtime_error(
                "Cannot find "
                +
                linkName);
        }
    }


    std::cout
        << "\nR2-B binding:\n"
        << "  M1Joint    -> joint index "
        << m1JointIndex_
        << "\n"
        << "  MotorShaft -> link index "
        << motorShaftLinkIndex_
        << "\n";


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        std::cout
            << "  TailJoint"
            << i
            << " -> joint index "
            << tailJointIndices_[i]
            << " | Tail"
            << i
            << " -> link index "
            << tailLinkIndices_[i]
            << "\n";
    }


    std::cout
        << std::endl;
}


int
TendonStageR2Simulator::FindJointIndex(
    const char* jointName) const
{
    if (
        fishDynamics_ == nullptr
        ||
        jointName == nullptr)
    {
        return -1;
    }


    const std::string
        shortName(
            jointName);


    const std::string
        fullName =
            "BionicFish/"
            +
            shortName;


    const unsigned int
        count =
            fishDynamics_->getNumOfJoints();


    for (
        unsigned int i = 0;
        i < count;
        ++i)
    {
        const std::string
            actual =
                fishDynamics_->getJointName(
                    i);


        if (
            actual == shortName
            ||
            actual == fullName)
        {
            return
                static_cast<int>(
                    i);
        }
    }


    return -1;
}


int
TendonStageR2Simulator::FindLinkIndex(
    const char* linkName) const
{
    if (
        fishDynamics_ == nullptr
        ||
        linkName == nullptr)
    {
        return -1;
    }


    const std::string
        shortName(
            linkName);


    const std::string
        fullName =
            "BionicFish/"
            +
            shortName;


    const unsigned int
        count =
            fishDynamics_->getNumOfLinks();


    // IMPORTANT:
    //
    // Do NOT use FeatherstoneRobot::getLinkIndex() here.
    //
    // That public Robot function uses Bullet-style indexing and
    // returns i-1 for non-base links.
    //
    // AddLinkForce() / getLinkTransform() require FeatherstoneEntity
    // indices:
    //
    //     Body = 0
    //     first child = 1
    //     ...
    //
    // Therefore search the FeatherstoneEntity link array directly.

    for (
        unsigned int i = 0;
        i < count;
        ++i)
    {
        const sf::FeatherstoneLink
            link =
                fishDynamics_->getLink(
                    i);


        if (
            link.solid == nullptr)
        {
            continue;
        }


        const std::string
            actual =
                link.solid->getName();


        if (
            actual == shortName
            ||
            actual == fullName)
        {
            return
                static_cast<int>(
                    i);
        }
    }


    return -1;
}


void
TendonStageR2Simulator::SetNeutralInitialCondition()
{
    fishDynamics_->setJointIC(
        static_cast<unsigned int>(
            m1JointIndex_),
        0.0,
        0.0);


    for (
        int index
        : tailJointIndices_)
    {
        fishDynamics_->setJointIC(
            static_cast<unsigned int>(
                index),
            0.0,
            0.0);
    }
}


void
TendonStageR2Simulator::RegisterTendonActuator()
{
    TendonTailParameters
        parameters;


    // ============================================================
    // Keep the baseline mechanical parameters unchanged for the
    // first R2-B geometry test.
    // ============================================================

    parameters.passiveStiffnessNmRad =
        0.65;


    parameters.passiveDampingNmsRad =
        0.0;


    parameters.tendonStiffnessNPerM =
        20000.0;


    parameters.tendonDampingNsPerM =
        10.0;


    parameters.initialPretensionN =
        0.0;


    parameters.maxDiagnosticStrain =
        0.03;


    parameters.motorTargetFrequencyHz =
        0.05;


    parameters.motorStartTimeS =
        1.0;


    parameters.motorRampTimeS =
        1.0;


    parameters.motorMaxTorqueNm =
        0.05;


    parameters.motorMaxVelocityRadS =
        0.5;


    parameters.jointSafetyLimitRad =
        1.0471975511965976;


    // ============================================================
    // Configure real Stonefish M1 Servo.
    // ============================================================

    m1Servo_->setControlMode(
        sf::ServoControlMode::VELOCITY);


    m1Servo_->setMaxTorque(
        parameters.motorMaxTorqueNm);


    m1Servo_->setMaxVelocity(
        parameters.motorMaxVelocityRadS);


    m1Servo_->setDesiredVelocity(
        0.0);


    // ============================================================
    // Convert indices to FeatherstoneEntity indices.
    // ============================================================

    std::array<unsigned int, 5>
        jointIndices {};


    std::array<unsigned int, 5>
        linkIndices {};


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        jointIndices[i] =
            static_cast<unsigned int>(
                tailJointIndices_[i]);


        linkIndices[i] =
            static_cast<unsigned int>(
                tailLinkIndices_[i]);
    }


    tendonActuator_ =
        new TendonTailActuator(
            "BionicFish/R2SpatialTendonDrive",
            fishDynamics_,
            m1Servo_,
            static_cast<unsigned int>(
                motorShaftLinkIndex_),
            linkIndices,
            jointIndices,
            parameters);


    AddActuator(
        tendonActuator_);
}


void
TendonStageR2Simulator::ConfigureCamera()
{
    sf::OpenGLTrackball*
        trackball =
            getTrackball();


    if (
        trackball == nullptr
        ||
        fishBody_ == nullptr)
    {
        return;
    }


    trackball->GlueToMoving(
        fishBody_);


    trackball->UpdateCenterPos();


    trackball->MouseScroll(
        -10.5f);


    trackball->UpdateTransform();
}


void
TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "tendon_stage_r2b.csv",
        std::ios::out
        |
        std::ios::trunc);


    if (
        !csv_.is_open())
    {
        throw std::runtime_error(
            "Cannot create tendon_stage_r2b.csv");
    }


    csv_
        << "time_s,"
        << "motor_cmd_rad_s,"
        << "motor_angle_rad,"
        << "motor_angle_deg,"
        << "motor_velocity_rad_s,"
        << "motor_effort_nm,"
        << "motor_saturated,"

        << "left_length_m,"
        << "right_length_m,"

        << "left_free_length_m,"
        << "right_free_length_m,"

        << "left_extension_m,"
        << "right_extension_m,"

        << "left_strain,"
        << "right_strain,"

        << "left_length_rate_m_s,"
        << "right_length_rate_m_s,"

        << "left_tension_n,"
        << "right_tension_n,"

        << "left_overstretch,"
        << "right_overstretch,";


    for (
        int i = 0;
        i < 5;
        ++i)
    {
        csv_
            << "j"
            << i
            << "_q_rad,"

            << "j"
            << i
            << "_q_deg,"

            << "j"
            << i
            << "_qdot_rad_s,"

            << "j"
            << i
            << "_passive_tau_nm,";
    }


    csv_
        << "safety_tripped\n";


    csv_
        << std::setprecision(
            10);
}


void
TendonStageR2Simulator::RecordSample()
{
    if (
        !csv_.is_open()
        ||
        tendonActuator_ == nullptr)
    {
        return;
    }


    const auto
        s =
            tendonActuator_->GetSnapshot();


    csv_
        << s.timeS
        << ","

        << s.motorCommandVelocityRadS
        << ","

        << s.motorAngleRad
        << ","

        << s.motorAngleRad
            *
            kRadToDeg
        << ","

        << s.motorVelocityRadS
        << ","

        << s.motorEffortTorqueNm
        << ","

        << (s.motorSaturated
                ?
            1
                :
            0)
        << ","

        << s.tendonLengthM[0]
        << ","

        << s.tendonLengthM[1]
        << ","

        << s.tendonFreeLengthM[0]
        << ","

        << s.tendonFreeLengthM[1]
        << ","

        << s.tendonExtensionM[0]
        << ","

        << s.tendonExtensionM[1]
        << ","

        << s.tendonStrain[0]
        << ","

        << s.tendonStrain[1]
        << ","

        << s.tendonLengthRateMS[0]
        << ","

        << s.tendonLengthRateMS[1]
        << ","

        << s.tendonTensionN[0]
        << ","

        << s.tendonTensionN[1]
        << ","

        << (s.tendonOverstretch[0]
                ?
            1
                :
            0)
        << ","

        << (s.tendonOverstretch[1]
                ?
            1
                :
            0)
        << ",";


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        csv_
            << s.jointPositionRad[i]
            << ","

            << s.jointPositionRad[i]
                *
                kRadToDeg
            << ","

            << s.jointVelocityRadS[i]
            << ","

            << s.passiveTorqueNm[i]
            << ",";
    }


    csv_
        << (s.safetyTripped
                ?
            1
                :
            0)
        << "\n";
}


void
TendonStageR2Simulator::PrintTelemetry()
{
    if (
        tendonActuator_ == nullptr)
    {
        return;
    }


    const auto
        s =
            tendonActuator_->GetSnapshot();


    std::cout
        << std::fixed
        << std::setprecision(
               3)

        << "[R2B] t="
        << s.timeS

        << " | M1="
        << s.motorAngleRad
            *
            kRadToDeg
        << "deg"

        << " cmd="
        << s.motorCommandVelocityRadS

        << " w="
        << s.motorVelocityRadS

        << " tau="
        << s.motorEffortTorqueNm
        << "Nm"

        << " | L=("
        << s.tendonLengthM[0]
            *
            1000.0
        << ","
        << s.tendonLengthM[1]
            *
            1000.0
        << ")mm"

        << " | T=("
        << s.tendonTensionN[0]
        << ","
        << s.tendonTensionN[1]
        << ")N"

        << " | strain=("
        << s.tendonStrain[0]
            *
            100.0
        << "%,"
        << s.tendonStrain[1]
            *
            100.0
        << "%)"

        << " | J=(";


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        if (
            i != 0)
        {
            std::cout
                << ",";
        }


        std::cout
            << s.jointPositionRad[i]
                *
                kRadToDeg;
    }


    std::cout
        << ")deg"

        << " | sat="
        << (s.motorSaturated
                ?
            "YES"
                :
            "NO")

        << " safety="
        << (s.safetyTripped
                ?
            "TRIPPED"
                :
            "OK")

        << std::endl;
}


void
TendonStageR2Simulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTimeS_ +=
        timeStep;


    if (
        lastCsvTimeS_ < 0.0
        ||
        elapsedTimeS_
            -
        lastCsvTimeS_
            >=
        csvPeriodS_)
    {
        RecordSample();


        lastCsvTimeS_ =
            elapsedTimeS_;
    }


    if (
        lastConsoleTimeS_ < 0.0
        ||
        elapsedTimeS_
            -
        lastConsoleTimeS_
            >=
        consolePeriodS_)
    {
        PrintTelemetry();


        lastConsoleTimeS_ =
            elapsedTimeS_;
    }
}
