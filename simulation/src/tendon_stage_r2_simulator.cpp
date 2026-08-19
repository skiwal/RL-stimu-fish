#include "inc/tendon_stage_r2_simulator.h"

#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/FeatherstoneEntity.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/graphics/OpenGLTrackball.h>

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr sf::Scalar kRadToDeg = 57.29577951308232;
}

TendonStageR2Simulator::TendonStageR2Simulator(sf::Scalar stepsPerSecond)
    : sf::SimulationManager(stepsPerSecond)
{
    setCallSimulationStepCompleted(true);
}

void TendonStageR2Simulator::BuildScenario()
{
    auto* app = sf::SimulationApp::getApp();
    if (!app)
        throw std::runtime_error("SimulationApp unavailable.");

    const std::string scenario = app->getDataPath() + "env/static_pool.scn";

    std::cout << "\nLoading:\n  " << scenario << "\n";

    sf::ScenarioParser parser(this);
    if (!parser.Parse(scenario))
        throw std::runtime_error("Failed to parse static_pool.scn");

    BindFish();
    SetNeutralInitialCondition();
    RegisterTendonActuator();
    OpenCsv();
    ConfigureCamera();

    std::cout
        << "\n============================================================\n"
        << " BionicFish Tethered Thrust Test\n"
        << "============================================================\n"
        << "Body fixed              YES\n"
        << "Motor                    DISABLED\n"
        << "Direct tendon tension    3 N peak\n"
        << "Tendon frequency         1.25 Hz\n"
        << "Routing                   0,0,0,1,1\n"
        << "Passive stiffness         0.65 Nm/rad\n"
        << "Hydro thrust              Fd + Ff\n"
        << "Forward direction         +X\n"
        << "Mean-thrust start         5.0 s\n"
        << "CSV                       tethered_thrust_test.csv\n"
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
        const std::string joint = "TailJoint" + std::to_string(i);
        const std::string link = "Tail" + std::to_string(i);

        tailJointIndices_[i] = FindJointIndex(joint.c_str());
        tailLinkIndices_[i] = FindLinkIndex(link.c_str());

        if (tailJointIndices_[i] < 0 || tailLinkIndices_[i] < 0)
            throw std::runtime_error("Cannot bind " + joint + "/" + link);

        tailSolids_[i] =
            fishDynamics_->getLink(
                static_cast<unsigned int>(tailLinkIndices_[i])).solid;

        if (!tailSolids_[i])
            throw std::runtime_error(link + " SolidEntity unavailable.");
    }

    caudalLinkIndex_ = FindLinkIndex("CaudalFin");
    if (caudalLinkIndex_ < 0)
        throw std::runtime_error("CaudalFin not found.");

    caudalFin_ =
        fishDynamics_->getLink(
            static_cast<unsigned int>(caudalLinkIndex_)).solid;

    if (!caudalFin_)
        throw std::runtime_error("CaudalFin SolidEntity unavailable.");

    std::cout << "\nThrust-test binding:\n";
    for (std::size_t i = 0; i < 5; ++i)
        std::cout << "  Tail" << i
                  << ": joint=" << tailJointIndices_[i]
                  << " link=" << tailLinkIndices_[i] << "\n";

    std::cout << "  CaudalFin: link=" << caudalLinkIndex_ << "\n\n";
}

int TendonStageR2Simulator::FindJointIndex(const char* name) const
{
    if (!fishDynamics_ || !name)
        return -1;

    const std::string shortName(name);
    const std::string fullName = "BionicFish/" + shortName;

    for (unsigned int i = 0; i < fishDynamics_->getNumOfJoints(); ++i)
    {
        const std::string actual = fishDynamics_->getJointName(i);
        if (actual == shortName || actual == fullName)
            return static_cast<int>(i);
    }

    return -1;
}

int TendonStageR2Simulator::FindLinkIndex(const char* name) const
{
    if (!fishDynamics_ || !name)
        return -1;

    const std::string shortName(name);
    const std::string fullName = "BionicFish/" + shortName;

    for (unsigned int i = 0; i < fishDynamics_->getNumOfLinks(); ++i)
    {
        const auto link = fishDynamics_->getLink(i);
        if (!link.solid)
            continue;

        const std::string actual = link.solid->getName();

        if (actual == shortName || actual == fullName)
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
}

void TendonStageR2Simulator::RegisterTendonActuator()
{
    TendonTailParameters p;

    p.passiveStiffnessNmRad = 0.65;
    p.passiveDampingNmsRad = 0.0;
    p.jointSafetyLimitRad = 1.0471975511965976; // 60 deg

    std::array<unsigned int,5> joints{};
    std::array<unsigned int,5> links{};

    for (std::size_t i = 0; i < 5; ++i)
    {
        joints[i] = static_cast<unsigned int>(tailJointIndices_[i]);
        links[i] = static_cast<unsigned int>(tailLinkIndices_[i]);
    }

    tendonActuator_ = new TendonTailActuator(
        "BionicFish/DirectTendonForceTest",
        fishDynamics_,
        static_cast<unsigned int>(bodyLinkIndex_),
        links,
        joints,
        p);

    AddActuator(tendonActuator_);
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

    // Propulsive hydrodynamic force:
    // form drag + skin friction.
    // Buoyancy is intentionally excluded.
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
    csv_.open("tethered_thrust_test.csv",
              std::ios::out | std::ios::trunc);

    if (!csv_.is_open())
        throw std::runtime_error("Cannot create tethered_thrust_test.csv");

    csv_
        << "time_s,phase,"
        << "left_tension_n,right_tension_n,"
        << "left_length_m,right_length_m,";

    for (int i = 0; i < 5; ++i)
        csv_ << "j" << i << "_deg,"
             << "j" << i << "_qdot_rad_s,";

    for (int i = 0; i < 5; ++i)
        csv_ << "tail" << i << "_hydro_fx_n,";

    csv_
        << "caudal_hydro_fx_n,"
        << "propulsor_fx_n,"
        << "propulsor_fy_n,"
        << "propulsor_fz_n,"
        << "mean_propulsor_fx_n,"
        << "mean_propulsor_fy_n,"
        << "mean_propulsor_fz_n,"
        << "safety_tripped\n";

    csv_ << std::setprecision(10);
}

void TendonStageR2Simulator::RecordSample()
{
    if (!csv_.is_open() || !tendonActuator_)
        return;

    const auto s = tendonActuator_->GetSnapshot();

    std::array<sf::Vector3,5> tailF;
    sf::Vector3 prop(0.0,0.0,0.0);

    for (std::size_t i = 0; i < 5; ++i)
    {
        tailF[i] = GetHydroForce(tailSolids_[i]);
        prop += tailF[i];
    }

    const sf::Vector3 caudalF = GetHydroForce(caudalFin_);
    prop += caudalF;

    const sf::Scalar meanFx =
        meanAccumTimeS_ > 0.0 ? impulseFxNs_ / meanAccumTimeS_ : 0.0;

    const sf::Scalar meanFy =
        meanAccumTimeS_ > 0.0 ? impulseFyNs_ / meanAccumTimeS_ : 0.0;

    const sf::Scalar meanFz =
        meanAccumTimeS_ > 0.0 ? impulseFzNs_ / meanAccumTimeS_ : 0.0;

    csv_
        << s.timeS << ","
        << s.testPhase << ","
        << s.commandedTensionN[0] << ","
        << s.commandedTensionN[1] << ","
        << s.tendonLengthM[0] << ","
        << s.tendonLengthM[1] << ",";

    for (std::size_t i = 0; i < 5; ++i)
        csv_
            << s.jointPositionRad[i] * kRadToDeg << ","
            << s.jointVelocityRadS[i] << ",";

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
        << (s.safetyTripped ? 1 : 0)
        << "\n";
}

void TendonStageR2Simulator::PrintTelemetry()
{
    if (!tendonActuator_)
        return;

    const auto s = tendonActuator_->GetSnapshot();
    const sf::Vector3 F = GetPropulsorForce();

    const sf::Scalar meanFx =
        meanAccumTimeS_ > 0.0 ? impulseFxNs_ / meanAccumTimeS_ : 0.0;

    const sf::Scalar meanFy =
        meanAccumTimeS_ > 0.0 ? impulseFyNs_ / meanAccumTimeS_ : 0.0;

    std::cout
        << std::fixed << std::setprecision(3)
        << "[THRUST] t=" << s.timeS
        << " T=(" << s.commandedTensionN[0]
        << "," << s.commandedTensionN[1] << ")N"
        << " F=(" << F.x()
        << "," << F.y()
        << "," << F.z() << ")N"
        << " meanFx=" << meanFx
        << " meanFy=" << meanFy
        << " J=(";

    for (std::size_t i = 0; i < 5; ++i)
    {
        if (i) std::cout << ",";
        std::cout << s.jointPositionRad[i] * kRadToDeg;
    }

    std::cout
        << ")deg safety="
        << (s.safetyTripped ? "TRIPPED" : "OK")
        << std::endl;
}

void TendonStageR2Simulator::SimulationStepCompleted(sf::Scalar timeStep)
{
    elapsedTimeS_ += timeStep;

    // Running average after startup transient.
    if (elapsedTimeS_ >= meanStartTimeS_)
    {
        const sf::Vector3 F = GetPropulsorForce();

        impulseFxNs_ += F.x() * timeStep;
        impulseFyNs_ += F.y() * timeStep;
        impulseFzNs_ += F.z() * timeStep;
        meanAccumTimeS_ += timeStep;
    }

    if (lastCsvTimeS_ < 0.0 ||
        elapsedTimeS_ - lastCsvTimeS_ >= csvPeriodS_)
    {
        RecordSample();
        lastCsvTimeS_ = elapsedTimeS_;
    }

    if (lastConsoleTimeS_ < 0.0 ||
        elapsedTimeS_ - lastConsoleTimeS_ >= consolePeriodS_)
    {
        PrintTelemetry();
        lastConsoleTimeS_ = elapsedTimeS_;
    }
}
