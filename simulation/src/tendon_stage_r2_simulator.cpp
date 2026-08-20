#include "inc/tendon_stage_r2_simulator.h"

#include <Stonefish/actuators/Actuator.h>
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

namespace {
constexpr sf::Scalar kRadToDeg = 57.29577951308232;

// First spring experiment.
constexpr sf::Scalar kCaudalK = 0.5;   // Nm/rad
constexpr sf::Scalar kCaudalC = 0.005;  // Nms/rad
}

// ============================================================
// Passive caudal-fin torsional spring
//
// tau = -k*q - c*qdot
//
// This is NOT a motor. MOTOR is only Stonefish's generic
// actuator category.
// ============================================================

class CaudalSpringActuator final : public sf::Actuator
{
public:
    CaudalSpringActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,
        unsigned int jointIndex,
        sf::Scalar stiffness,
        sf::Scalar damping)
        : sf::Actuator(name),
          dynamics_(dynamics),
          jointIndex_(jointIndex),
          stiffness_(stiffness),
          damping_(damping)
    {
    }

    sf::ActuatorType getType() const override
    {
        return sf::ActuatorType::MOTOR;
    }

    void Update(sf::Scalar timeStep) override
    {
        sf::Actuator::Update(timeStep);

        if (!dynamics_)
        {
            valid_ = false;
            return;
        }

        btMultibodyLink::eFeatherstoneJointType positionType =
            btMultibodyLink::eInvalid;
        btMultibodyLink::eFeatherstoneJointType velocityType =
            btMultibodyLink::eInvalid;

        dynamics_->getJointPosition(
            jointIndex_, position_, positionType);

        dynamics_->getJointVelocity(
            jointIndex_, velocity_, velocityType);

        valid_ =
            positionType == btMultibodyLink::eRevolute &&
            velocityType == btMultibodyLink::eRevolute &&
            std::isfinite(static_cast<double>(position_)) &&
            std::isfinite(static_cast<double>(velocity_));

        if (!valid_)
        {
            torque_ = 0.0;
            return;
        }

        torque_ =
            -stiffness_ * position_
            -damping_ * velocity_;

        dynamics_->DriveJoint(jointIndex_, torque_);
    }

    sf::Scalar Position() const { return position_; }
    sf::Scalar Velocity() const { return velocity_; }
    sf::Scalar Torque() const { return torque_; }
    bool Valid() const { return valid_; }

private:
    sf::FeatherstoneEntity* dynamics_ = nullptr;
    unsigned int jointIndex_ = 0;

    sf::Scalar stiffness_ = 0.0;
    sf::Scalar damping_ = 0.0;

    sf::Scalar position_ = 0.0;
    sf::Scalar velocity_ = 0.0;
    sf::Scalar torque_ = 0.0;

    bool valid_ = false;
};

// ============================================================

TendonStageR2Simulator::TendonStageR2Simulator(
    sf::Scalar stepsPerSecond)
    : sf::SimulationManager(stepsPerSecond)
{
    setCallSimulationStepCompleted(true);
}

void TendonStageR2Simulator::BuildScenario()
{
    auto* app = sf::SimulationApp::getApp();

    if (!app)
        throw std::runtime_error("SimulationApp unavailable.");

    const std::string scenario =
        app->getDataPath() + "env/static_pool.scn";

    std::cout << "\nLoading:\n  " << scenario << "\n";

    sf::ScenarioParser parser(this);

    if (!parser.Parse(scenario))
        throw std::runtime_error("Failed to parse static_pool.scn");

    BindFish();
    SetNeutralInitialCondition();
    RegisterTendonActuator();
    RegisterCaudalSpringActuator();
    OpenCsv();
    ConfigureCamera();

    std::cout
        << "\n============================================================\n"
        << " BionicFish Caudal Spring Thrust Test\n"
        << "============================================================\n"
        << "Body fixed              YES\n"
        << "Motor                   DISABLED\n"
        << "Tendon peak             3 N\n"
        << "Tendon frequency        0.60 Hz\n"
        << "Tail stiffness          0.65 Nm/rad\n"
        << "Caudal joint            REVOLUTE +/-20 deg\n"
        << "Caudal spring k         " << kCaudalK << " Nm/rad\n"
        << "Caudal damping c        " << kCaudalC << " Nms/rad\n"
        << "Forward                 +X\n"
        << "Mean start              5.0 s\n"
        << "CSV                     tethered_thrust_test.csv\n"
        << "============================================================\n\n";
}

void TendonStageR2Simulator::BindFish()
{
    fishRobot_ = getRobot("BionicFish");

    if (!fishRobot_)
        throw std::runtime_error("BionicFish not found.");

    fishFeatherstoneRobot_ =
        dynamic_cast<sf::FeatherstoneRobot*>(fishRobot_);

    if (!fishFeatherstoneRobot_)
        throw std::runtime_error("BionicFish is not FeatherstoneRobot.");

    fishDynamics_ = fishFeatherstoneRobot_->getDynamics();
    fishBody_ = fishRobot_->getBaseLink();

    if (!fishDynamics_ || !fishBody_)
        throw std::runtime_error("Fish dynamics/base unavailable.");

    bodyLinkIndex_ = FindLinkIndex("Body");

    if (bodyLinkIndex_ < 0)
        throw std::runtime_error("Body link not found.");

    for (std::size_t i = 0; i < 5; ++i)
    {
        const std::string joint =
            "TailJoint" + std::to_string(i);

        const std::string link =
            "Tail" + std::to_string(i);

        tailJointIndices_[i] =
            FindJointIndex(joint.c_str());

        tailLinkIndices_[i] =
            FindLinkIndex(link.c_str());

        if (tailJointIndices_[i] < 0 ||
            tailLinkIndices_[i] < 0)
            throw std::runtime_error(
                "Cannot bind " + joint + "/" + link);

        tailSolids_[i] =
            fishDynamics_
                ->getLink(
                    static_cast<unsigned int>(
                        tailLinkIndices_[i]))
                .solid;

        if (!tailSolids_[i])
            throw std::runtime_error(
                link + " SolidEntity unavailable.");
    }

    caudalJointIndex_ = FindJointIndex("CaudalJoint");
    caudalLinkIndex_ = FindLinkIndex("CaudalFin");

    if (caudalJointIndex_ < 0 || caudalLinkIndex_ < 0)
        throw std::runtime_error(
            "CaudalJoint/CaudalFin not found.");

    caudalFin_ =
        fishDynamics_
            ->getLink(
                static_cast<unsigned int>(
                    caudalLinkIndex_))
            .solid;

    if (!caudalFin_)
        throw std::runtime_error(
            "CaudalFin SolidEntity unavailable.");

    std::cout << "\nHydro binding:\n";

    for (std::size_t i = 0; i < 5; ++i)
        std::cout
            << "  Tail" << i
            << " joint=" << tailJointIndices_[i]
            << " link=" << tailLinkIndices_[i]
            << "\n";

    std::cout
        << "  CaudalJoint joint=" << caudalJointIndex_
        << "\n"
        << "  CaudalFin link=" << caudalLinkIndex_
        << "\n\n";
}

int TendonStageR2Simulator::FindJointIndex(
    const char* name) const
{
    if (!fishDynamics_ || !name)
        return -1;

    const std::string shortName(name);
    const std::string fullName =
        "BionicFish/" + shortName;

    for (unsigned int i = 0;
         i < fishDynamics_->getNumOfJoints();
         ++i)
    {
        const std::string actual =
            fishDynamics_->getJointName(i);

        if (actual == shortName ||
            actual == fullName)
            return static_cast<int>(i);
    }

    return -1;
}

int TendonStageR2Simulator::FindLinkIndex(
    const char* name) const
{
    if (!fishDynamics_ || !name)
        return -1;

    const std::string shortName(name);
    const std::string fullName =
        "BionicFish/" + shortName;

    for (unsigned int i = 0;
         i < fishDynamics_->getNumOfLinks();
         ++i)
    {
        const auto link =
            fishDynamics_->getLink(i);

        if (!link.solid)
            continue;

        const std::string actual =
            link.solid->getName();

        if (actual == shortName ||
            actual == fullName)
            return static_cast<int>(i);
    }

    return -1;
}

void TendonStageR2Simulator::SetNeutralInitialCondition()
{
    for (int index : tailJointIndices_)
        fishDynamics_->setJointIC(
            static_cast<unsigned int>(index),
            0.0,
            0.0);

    fishDynamics_->setJointIC(
        static_cast<unsigned int>(caudalJointIndex_),
        0.0,
        0.0);
}

void TendonStageR2Simulator::RegisterTendonActuator()
{
    TendonTailParameters p;

    p.passiveStiffnessNmRad = 0.65;
    p.passiveDampingNmsRad = 0.0;
    p.jointSafetyLimitRad = 1.0471975511965976;

    std::array<unsigned int, 5> joints{};
    std::array<unsigned int, 5> links{};

    for (std::size_t i = 0; i < 5; ++i)
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
            "BionicFish/DirectTendonForceTest",
            fishDynamics_,
            static_cast<unsigned int>(bodyLinkIndex_),
            links,
            joints,
            p);

    AddActuator(tendonActuator_);
}

void TendonStageR2Simulator::RegisterCaudalSpringActuator()
{
    caudalSpringActuator_ =
        new CaudalSpringActuator(
            "BionicFish/CaudalPassiveSpring",
            fishDynamics_,
            static_cast<unsigned int>(
                caudalJointIndex_),
            kCaudalK,
            kCaudalC);

    AddActuator(caudalSpringActuator_);
}

void TendonStageR2Simulator::ConfigureCamera()
{
    auto* trackball = getTrackball();

    if (!trackball || !fishBody_)
        return;

    trackball->GlueToMoving(fishBody_);
    trackball->UpdateCenterPos();
    trackball->MouseScroll(-10.5f);
    trackball->UpdateTransform();
}

sf::Vector3 TendonStageR2Simulator::GetHydroForce(
    sf::SolidEntity* solid) const
{
    if (!solid)
        return sf::Vector3(0.0, 0.0, 0.0);

    sf::Vector3 Fb, Tb, Fd, Td, Ff, Tf;

    solid->getHydrodynamicForces(
        Fb, Tb,
        Fd, Td,
        Ff, Tf);

    return Fd + Ff;
}

sf::Vector3 TendonStageR2Simulator::GetPropulsorForce() const
{
    sf::Vector3 total(0.0, 0.0, 0.0);

    for (auto* tail : tailSolids_)
        total += GetHydroForce(tail);

    total += GetHydroForce(caudalFin_);

    return total;
}

void TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "tethered_thrust_test.csv",
        std::ios::out | std::ios::trunc);

    if (!csv_.is_open())
        throw std::runtime_error(
            "Cannot create tethered_thrust_test.csv");

    csv_
        << "time_s,phase,"
        << "left_tension_n,right_tension_n,"
        << "left_length_m,right_length_m,";

    for (int i = 0; i < 5; ++i)
        csv_
            << "j" << i << "_deg,"
            << "j" << i << "_qdot_rad_s,";

    csv_
        << "caudal_deg,"
        << "caudal_qdot_rad_s,"
        << "caudal_spring_torque_nm,";

    for (int i = 0; i < 5; ++i)
        csv_
            << "tail" << i
            << "_hydro_fx_n,";

    csv_
        << "caudal_hydro_fx_n,"
        << "propulsor_fx_n,"
        << "propulsor_fy_n,"
        << "propulsor_fz_n,"
        << "mean_propulsor_fx_n,"
        << "mean_propulsor_fy_n,"
        << "mean_propulsor_fz_n,"
        << "tendon_safety_tripped,"
        << "caudal_valid\n";

    csv_ << std::setprecision(10);
}

void TendonStageR2Simulator::RecordSample()
{
    if (!csv_.is_open() ||
        !tendonActuator_ ||
        !caudalSpringActuator_)
        return;

    const auto s =
        tendonActuator_->GetSnapshot();

    std::array<sf::Vector3, 5> tailF;
    sf::Vector3 prop(0.0, 0.0, 0.0);

    for (std::size_t i = 0; i < 5; ++i)
    {
        tailF[i] =
            GetHydroForce(tailSolids_[i]);

        prop += tailF[i];
    }

    const sf::Vector3 caudalF =
        GetHydroForce(caudalFin_);

    prop += caudalF;

    const sf::Scalar meanFx =
        meanAccumTimeS_ > 0.0
            ? impulseFxNs_ / meanAccumTimeS_
            : 0.0;

    const sf::Scalar meanFy =
        meanAccumTimeS_ > 0.0
            ? impulseFyNs_ / meanAccumTimeS_
            : 0.0;

    const sf::Scalar meanFz =
        meanAccumTimeS_ > 0.0
            ? impulseFzNs_ / meanAccumTimeS_
            : 0.0;

    csv_
        << s.timeS << ","
        << s.testPhase << ","
        << s.commandedTensionN[0] << ","
        << s.commandedTensionN[1] << ","
        << s.tendonLengthM[0] << ","
        << s.tendonLengthM[1] << ",";

    for (std::size_t i = 0; i < 5; ++i)
        csv_
            << s.jointPositionRad[i]
                * kRadToDeg << ","
            << s.jointVelocityRadS[i]
            << ",";

    csv_
        << caudalSpringActuator_->Position()
            * kRadToDeg << ","
        << caudalSpringActuator_->Velocity()
        << ","
        << caudalSpringActuator_->Torque()
        << ",";

    for (std::size_t i = 0; i < 5; ++i)
        csv_ << tailF[i].x() << ",";

    csv_
        << caudalF.x() << ","
        << prop.x() << ","
        << prop.y() << ","
        << prop.z() << ","
        << meanFx << ","
        << meanFy << ","
        << meanFz << ","
        << (s.safetyTripped ? 1 : 0) << ","
        << (caudalSpringActuator_->Valid() ? 1 : 0)
        << "\n";
}

void TendonStageR2Simulator::PrintTelemetry()
{
    if (!tendonActuator_ ||
        !caudalSpringActuator_)
        return;

    const auto s =
        tendonActuator_->GetSnapshot();

    const sf::Vector3 F =
        GetPropulsorForce();

    const sf::Scalar meanFx =
        meanAccumTimeS_ > 0.0
            ? impulseFxNs_ / meanAccumTimeS_
            : 0.0;

    std::cout
        << std::fixed
        << std::setprecision(3)
        << "[HYDRO] t=" << s.timeS
        << " T=("
        << s.commandedTensionN[0]
        << ","
        << s.commandedTensionN[1]
        << ")"
        << " F=("
        << F.x() << ","
        << F.y() << ","
        << F.z() << ")"
        << " meanFx=" << meanFx
        << " J=(";

    for (std::size_t i = 0; i < 5; ++i)
    {
        if (i)
            std::cout << ",";

        std::cout
            << s.jointPositionRad[i]
                * kRadToDeg;
    }

    std::cout
        << ") C=("
        << caudalSpringActuator_->Position()
            * kRadToDeg
        << "deg,"
        << caudalSpringActuator_->Velocity()
        << "rad/s,"
        << caudalSpringActuator_->Torque()
        << "Nm)"
        << " safety="
        << (s.safetyTripped
                ? "TRIPPED"
                : "OK")
        << " caudal="
        << (caudalSpringActuator_->Valid()
                ? "OK"
                : "INVALID")
        << std::endl;
}

void TendonStageR2Simulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTimeS_ += timeStep;

    if (elapsedTimeS_ >= meanStartTimeS_)
    {
        const sf::Vector3 F =
            GetPropulsorForce();

        impulseFxNs_ += F.x() * timeStep;
        impulseFyNs_ += F.y() * timeStep;
        impulseFzNs_ += F.z() * timeStep;
        meanAccumTimeS_ += timeStep;
    }

    if (lastCsvTimeS_ < 0.0 ||
        elapsedTimeS_ - lastCsvTimeS_
            >= csvPeriodS_)
    {
        RecordSample();
        lastCsvTimeS_ = elapsedTimeS_;
    }

    if (lastConsoleTimeS_ < 0.0 ||
        elapsedTimeS_ - lastConsoleTimeS_
            >= consolePeriodS_)
    {
        PrintTelemetry();
        lastConsoleTimeS_ = elapsedTimeS_;
    }
}
