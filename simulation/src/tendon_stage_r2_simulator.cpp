#include "inc/tendon_stage_r2_simulator.h"
#include "inc/tendon_tail_actuator.h"

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

void TendonStageR2Simulator::BuildScenario()
{
    sf::SimulationApp *app =
        sf::SimulationApp::getApp();

    if (
        app == nullptr)
    {
        throw std::runtime_error(
            "TendonStageR2Simulator: SimulationApp unavailable.");
    }

    const std::string
        scenario =
            app->getDataPath() +
            "env/static_pool.scn";

    std::cout
        << "\nLoading:\n  "
        << scenario
        << "\n";

    sf::ScenarioParser parser(
        this);

    if (
        !parser.Parse(
            scenario))
    {
        throw std::runtime_error(
            "Failed to parse static_pool.scn");
    }

    BindFish();

    SetNeutralTailInitialCondition();

    RegisterTendonActuator();

    OpenCsv();

    ConfigureCamera();

    const auto
        snapshot =
            tendonActuator_->GetSnapshot();

    std::cout
        << "\n"
        << "============================================================\n"
        << " BionicFish Stage R2-A\n"
        << " Kinematic M1 + Generalized Two-Tendon Drive\n"
        << "============================================================\n"
        << "Body fixed                 YES\n"
        << "Tail joints                J0~J4 passive\n"
        << "Native CableEntity         NO\n"
        << "Tendon model               generalized-coordinate\n"
        << "Routing                    0,0,0,1,1\n"
        << "Tendon stiffness           20000 N/m\n"
        << "Tendon damping             10 N*s/m\n"
        << "Passive joint stiffness    0.65 Nm/rad\n"
        << "Motor diagnostic freq      0.05 Hz\n"
        << "Motor stall safety         6.9 Nm\n"
        << "Left rest length           "
        << snapshot.tendonRestLengthM[0]
        << " m\n"
        << "Right rest length          "
        << snapshot.tendonRestLengthM[1]
        << " m\n"
        << "CSV                         tendon_stage_r2a.csv\n"
        << "============================================================\n"
        << std::endl;
}

void TendonStageR2Simulator::BindFish()
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
        dynamic_cast<sf::FeatherstoneRobot *>(
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
        fishDynamics_ == nullptr ||
        fishBody_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish dynamics/base unavailable.");
    }

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        const std::string
            name =
                "TailJoint" +
                std::to_string(
                    i);

        tailJointIndices_[i] =
            FindJointIndex(
                name.c_str());

        if (
            tailJointIndices_[i] < 0)
        {
            throw std::runtime_error(
                "Cannot find " +
                name);
        }
    }

    std::cout
        << "\nR2 joint binding:\n";

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        std::cout
            << "  J"
            << i
            << " -> dynamics index "
            << tailJointIndices_[i]
            << "\n";
    }
}

int TendonStageR2Simulator::FindJointIndex(
    const char *jointName) const
{
    if (
        fishDynamics_ == nullptr ||
        jointName == nullptr)
    {
        return -1;
    }

    const std::string
        shortName(
            jointName);

    const std::string
        fullName =
            "BionicFish/" +
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
            actual == shortName ||
            actual == fullName)
        {
            return static_cast<int>(
                i);
        }
    }

    return -1;
}

void TendonStageR2Simulator::SetNeutralTailInitialCondition()
{
    for (
        int index : tailJointIndices_)
    {
        fishDynamics_->setJointIC(
            static_cast<unsigned int>(
                index),
            0.0,
            0.0);
    }
}

void TendonStageR2Simulator::RegisterTendonActuator()
{
    std::array<unsigned int, 5>
        indices{};

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        indices[i] =
            static_cast<unsigned int>(
                tailJointIndices_[i]);
    }

    TendonTailParameters
        parameters;

    /*
        Do NOT change these values yet.

        This first run is intended to test topology and tendon
        geometry, not tune performance.
    */

    parameters.passiveStiffnessNmRad =
        0.65;

    parameters.passiveDampingNmsRad =
        0.0;

    parameters.tendonStiffnessNPerM =
        20000.0;

    parameters.tendonDampingNsPerM =
        10.0;

    parameters.motorTargetFrequencyHz =
        0.05;

    parameters.motorStartTimeS =
        1.0;

    parameters.motorRampTimeS =
        1.0;

    parameters.motorReactionTorqueLimitNm =
        6.9;

    tendonActuator_ =
        new TendonTailActuator(
            "BionicFish/R2GeneralizedTendonDrive",
            fishDynamics_,
            indices,
            parameters);

    AddActuator(
        tendonActuator_);
}

void TendonStageR2Simulator::ConfigureCamera()
{
    sf::OpenGLTrackball *
        trackball =
            getTrackball();

    if (
        trackball == nullptr ||
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

void TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "tendon_stage_r2a.csv",
        std::ios::out |
            std::ios::trunc);

    if (
        !csv_.is_open())
    {
        throw std::runtime_error(
            "Cannot create tendon_stage_r2a.csv");
    }

    csv_
        << "time_s,"
        << "motor_angle_rad,"
        << "motor_angle_deg,"
        << "motor_velocity_rad_s,"
        << "motor_required_torque_nm,"

        << "left_length_m,"
        << "right_length_m,"
        << "left_rest_m,"
        << "right_rest_m,"
        << "left_stretch_m,"
        << "right_stretch_m,"
        << "left_length_rate_m_s,"
        << "right_length_rate_m_s,"
        << "left_force_n,"
        << "right_force_n,";

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
            << "_passive_tau_nm,"
            << "j"
            << i
            << "_tendon_tau_nm,"
            << "j"
            << i
            << "_total_tau_nm,";
    }

    csv_
        << "safety_tripped\n";

    csv_
        << std::setprecision(
               10);
}

void TendonStageR2Simulator::RecordSample()
{
    if (
        !csv_.is_open() ||
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
        << s.motorAngleRad
        << ","
        << s.motorAngleRad * kRadToDeg
        << ","
        << s.motorVelocityRadS
        << ","
        << s.motorRequiredTorqueNm
        << ","

        << s.tendonLengthM[0]
        << ","
        << s.tendonLengthM[1]
        << ","
        << s.tendonRestLengthM[0]
        << ","
        << s.tendonRestLengthM[1]
        << ","
        << s.tendonStretchM[0]
        << ","
        << s.tendonStretchM[1]
        << ","
        << s.tendonLengthRateMS[0]
        << ","
        << s.tendonLengthRateMS[1]
        << ","
        << s.tendonForceN[0]
        << ","
        << s.tendonForceN[1]
        << ",";

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        csv_
            << s.jointPositionRad[i]
            << ","
            << s.jointPositionRad[i] * kRadToDeg
            << ","
            << s.jointVelocityRadS[i]
            << ","
            << s.passiveTorqueNm[i]
            << ","
            << s.tendonTorqueNm[i]
            << ","
            << s.totalTorqueNm[i]
            << ",";
    }

    csv_
        << (s.safetyTripped
                ? 1
                : 0)
        << "\n";
}

void TendonStageR2Simulator::PrintTelemetry()
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

        << "[R2A] t="
        << s.timeS

        << " | M1="
        << s.motorAngleRad * kRadToDeg
        << "deg"

        << " w="
        << s.motorVelocityRadS

        << "rad/s"

        << " tauReq="
        << s.motorRequiredTorqueNm
        << "Nm"

        << " | T=("
        << s.tendonForceN[0]
        << ","
        << s.tendonForceN[1]
        << ")N"

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
            << s.jointPositionRad[i] * kRadToDeg;
    }

    std::cout
        << ")deg"

        << " | safety="
        << (s.safetyTripped
                ? "TRIPPED"
                : "OK")

        << std::endl;
}

void TendonStageR2Simulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTimeS_ +=
        timeStep;

    if (
        lastCsvTimeS_ < 0.0 ||
        elapsedTimeS_ - lastCsvTimeS_ >=
            csvPeriodS_)
    {
        RecordSample();

        lastCsvTimeS_ =
            elapsedTimeS_;
    }

    if (
        lastConsoleTimeS_ < 0.0 ||
        elapsedTimeS_ - lastConsoleTimeS_ >=
            consolePeriodS_)
    {
        PrintTelemetry();

        lastConsoleTimeS_ =
            elapsedTimeS_;
    }
}
