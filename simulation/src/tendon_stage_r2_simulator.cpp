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

TendonStageR2Simulator::TendonStageR2Simulator(sf::Scalar hz)
    : sf::SimulationManager(hz)
{
    setCallSimulationStepCompleted(true);
}

void TendonStageR2Simulator::BuildScenario()
{
    auto* app = sf::SimulationApp::getApp();
    if (!app) throw std::runtime_error("SimulationApp unavailable.");

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
        << " BionicFish Tethered Hydro Test\n"
        << "============================================================\n"
        << "Body fixed              YES\n"
        << "Motor                    DISABLED\n"
        << "Tendon peak              3 N\n"
        << "Frequency                1.25 Hz\n"
        << "Period                   0.80 s\n"
        << "Routing                   0,0,0,1,1\n"
        << "Forward                   +X\n"
        << "Mean start               5.0 s\n"
        << "CSV                       tethered_thrust_test.csv\n"
        << "============================================================\n\n";
}

void TendonStageR2Simulator::BindFish()
{
    fishRobot_ = getRobot("BionicFish");
    if (!fishRobot_) throw std::runtime_error("BionicFish not found.");

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
        throw std::runtime_error("Body not found.");

    for (std::size_t i=0; i<5; ++i) {
        const std::string j = "TailJoint" + std::to_string(i);
        const std::string l = "Tail" + std::to_string(i);

        tailJointIndices_[i] = FindJointIndex(j.c_str());
        tailLinkIndices_[i] = FindLinkIndex(l.c_str());

        if (tailJointIndices_[i] < 0 || tailLinkIndices_[i] < 0)
            throw std::runtime_error("Cannot bind " + j + "/" + l);

        tailSolids_[i] =
            fishDynamics_->getLink(
                static_cast<unsigned int>(tailLinkIndices_[i])).solid;

        if (!tailSolids_[i])
            throw std::runtime_error(l + " solid unavailable.");
    }

    caudalLinkIndex_ = FindLinkIndex("CaudalFin");
    if (caudalLinkIndex_ < 0)
        throw std::runtime_error("CaudalFin not found.");

    caudalFin_ =
        fishDynamics_->getLink(
            static_cast<unsigned int>(caudalLinkIndex_)).solid;

    if (!caudalFin_)
        throw std::runtime_error("CaudalFin solid unavailable.");

    std::cout << "\nHydro binding:\n";
    for (std::size_t i=0; i<5; ++i)
        std::cout << " Tail" << i
                  << " joint=" << tailJointIndices_[i]
                  << " link=" << tailLinkIndices_[i] << "\n";

    std::cout << " CaudalFin link=" << caudalLinkIndex_ << "\n\n";
}

int TendonStageR2Simulator::FindJointIndex(const char* name) const
{
    if (!fishDynamics_ || !name) return -1;

    const std::string a(name), b="BionicFish/"+a;

    for (unsigned int i=0; i<fishDynamics_->getNumOfJoints(); ++i) {
        const std::string n = fishDynamics_->getJointName(i);
        if (n==a || n==b) return static_cast<int>(i);
    }

    return -1;
}

int TendonStageR2Simulator::FindLinkIndex(const char* name) const
{
    if (!fishDynamics_ || !name) return -1;

    const std::string a(name), b="BionicFish/"+a;

    for (unsigned int i=0; i<fishDynamics_->getNumOfLinks(); ++i) {
        const auto link = fishDynamics_->getLink(i);
        if (!link.solid) continue;

        const std::string n = link.solid->getName();
        if (n==a || n==b) return static_cast<int>(i);
    }

    return -1;
}

void TendonStageR2Simulator::SetNeutralInitialCondition()
{
    for (int j : tailJointIndices_)
        fishDynamics_->setJointIC(
            static_cast<unsigned int>(j),0.0,0.0);
}

void TendonStageR2Simulator::RegisterTendonActuator()
{
    TendonTailParameters p;

    p.passiveStiffnessNmRad = 0.65;
    p.passiveDampingNmsRad = 0.0;
    p.directTestTensionN = 3.0;
    p.initialSettleTimeS = 1.0;
    p.rampTimeS = 1.0;
    p.jointSafetyLimitRad = 1.0471975511965976;

    std::array<unsigned int,5> joints{}, links{};

    for (std::size_t i=0; i<5; ++i) {
        joints[i] = static_cast<unsigned int>(tailJointIndices_[i]);
        links[i] = static_cast<unsigned int>(tailLinkIndices_[i]);
    }

    tendonActuator_ = new TendonTailActuator(
        "BionicFish/DirectTendonForceTest",
        fishDynamics_,
        static_cast<unsigned int>(bodyLinkIndex_),
        links,joints,p);

    AddActuator(tendonActuator_);
}

void TendonStageR2Simulator::ConfigureCamera()
{
    auto* camera = getTrackball();
    if (!camera || !fishBody_) return;

    camera->GlueToMoving(fishBody_);
    camera->UpdateCenterPos();
    camera->MouseScroll(-10.5f);
    camera->UpdateTransform();
}

TendonStageR2Simulator::HydroForce
TendonStageR2Simulator::GetHydroForce(sf::SolidEntity* solid) const
{
    HydroForce h;
    if (!solid) return h;

    sf::Vector3 Fb,Tb,Fd,Td,Ff,Tf;

    solid->getHydrodynamicForces(
        Fb,Tb,Fd,Td,Ff,Tf);

    // Explicit Stonefish hydrodynamic force.
    h.drag = Fd + Ff;

    // Stonefish translational augmented mass:
    // M_aug = M_body + M_added.
    //
    // Reconstruct the equivalent reactive fluid force:
    // F_reactive = -M_added * a.
    //
    // DIAGNOSTIC ONLY. Never apply it back into Stonefish.
    const sf::Scalar addedMass =
        solid->getAugmentedMass() - solid->getMass();

    h.reactive =
        solid->getLinearAcceleration() * (-addedMass);

    h.total = h.drag + h.reactive;
    return h;
}

TendonStageR2Simulator::HydroForce
TendonStageR2Simulator::GetPropulsorForce() const
{
    HydroForce out;

    for (auto* solid : tailSolids_) {
        const HydroForce h = GetHydroForce(solid);
        out.drag += h.drag;
        out.reactive += h.reactive;
        out.total += h.total;
    }

    const HydroForce fin = GetHydroForce(caudalFin_);
    out.drag += fin.drag;
    out.reactive += fin.reactive;
    out.total += fin.total;

    return out;
}

void TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "tethered_thrust_test.csv",
        std::ios::out | std::ios::trunc);

    if (!csv_.is_open())
        throw std::runtime_error("Cannot create tethered_thrust_test.csv");

    csv_
        << "time_s,phase,left_tension_n,right_tension_n,"
        << "left_length_m,right_length_m,";

    for (int i=0; i<5; ++i)
        csv_ << "j" << i << "_deg,"
             << "j" << i << "_qdot_rad_s,";

    for (int i=0; i<5; ++i)
        csv_ << "tail" << i << "_drag_fx_n,";

    csv_
        << "caudal_drag_fx_n,"
        << "drag_fx_n,"
        << "reactive_fx_n,"
        << "total_fx_n,"
        << "total_fy_n,"
        << "total_fz_n,"
        << "mean_drag_fx_n,"
        << "mean_reactive_fx_n,"
        << "mean_total_fx_n,"
        << "mean_total_fy_n,"
        << "mean_total_fz_n,"
        << "safety_tripped\n";

    csv_ << std::setprecision(10);
}

void TendonStageR2Simulator::RecordSample()
{
    if (!csv_.is_open() || !tendonActuator_) return;

    const auto s = tendonActuator_->GetSnapshot();

    std::array<HydroForce,5> tail;
    for (std::size_t i=0; i<5; ++i)
        tail[i] = GetHydroForce(tailSolids_[i]);

    const HydroForce fin = GetHydroForce(caudalFin_);
    const HydroForce hydro = GetPropulsorForce();

    const sf::Scalar meanDrag =
        meanTimeS_>0.0 ? dragImpulseFx_/meanTimeS_ : 0.0;

    const sf::Scalar meanReactive =
        meanTimeS_>0.0 ? reactiveImpulseFx_/meanTimeS_ : 0.0;

    const sf::Scalar meanFx =
        meanTimeS_>0.0 ? totalImpulseFx_/meanTimeS_ : 0.0;

    const sf::Scalar meanFy =
        meanTimeS_>0.0 ? totalImpulseFy_/meanTimeS_ : 0.0;

    const sf::Scalar meanFz =
        meanTimeS_>0.0 ? totalImpulseFz_/meanTimeS_ : 0.0;

    csv_
        << s.timeS << ","
        << s.testPhase << ","
        << s.commandedTensionN[0] << ","
        << s.commandedTensionN[1] << ","
        << s.tendonLengthM[0] << ","
        << s.tendonLengthM[1] << ",";

    for (std::size_t i=0; i<5; ++i)
        csv_ << s.jointPositionRad[i]*kRadToDeg << ","
             << s.jointVelocityRadS[i] << ",";

    for (std::size_t i=0; i<5; ++i)
        csv_ << tail[i].drag.x() << ",";

    csv_
        << fin.drag.x() << ","
        << hydro.drag.x() << ","
        << hydro.reactive.x() << ","
        << hydro.total.x() << ","
        << hydro.total.y() << ","
        << hydro.total.z() << ","
        << meanDrag << ","
        << meanReactive << ","
        << meanFx << ","
        << meanFy << ","
        << meanFz << ","
        << (s.safetyTripped ? 1 : 0)
        << "\n";
}

void TendonStageR2Simulator::PrintTelemetry()
{
    if (!tendonActuator_) return;

    const auto s = tendonActuator_->GetSnapshot();
    const HydroForce h = GetPropulsorForce();

    const sf::Scalar meanDrag =
        meanTimeS_>0.0 ? dragImpulseFx_/meanTimeS_ : 0.0;

    const sf::Scalar meanReactive =
        meanTimeS_>0.0 ? reactiveImpulseFx_/meanTimeS_ : 0.0;

    const sf::Scalar meanFx =
        meanTimeS_>0.0 ? totalImpulseFx_/meanTimeS_ : 0.0;

    const sf::Scalar meanFy =
        meanTimeS_>0.0 ? totalImpulseFy_/meanTimeS_ : 0.0;

    std::cout
        << std::fixed << std::setprecision(3)
        << "[HYDRO] t=" << s.timeS
        << " T=(" << s.commandedTensionN[0]
        << "," << s.commandedTensionN[1] << ")"
        << " dragFx=" << h.drag.x()
        << " reactiveFx=" << h.reactive.x()
        << " totalFx=" << h.total.x()
        << " | mean=("
        << meanDrag << ","
        << meanReactive << ","
        << meanFx << ")"
        << " meanFy=" << meanFy
        << " | J=(";

    for (std::size_t i=0; i<5; ++i) {
        if (i) std::cout << ",";
        std::cout << s.jointPositionRad[i]*kRadToDeg;
    }

    std::cout
        << ") safety="
        << (s.safetyTripped ? "TRIPPED" : "OK")
        << "\n";
}

void TendonStageR2Simulator::SimulationStepCompleted(sf::Scalar dt)
{
    elapsedTimeS_ += dt;

    if (elapsedTimeS_ >= meanStartTimeS_) {
        const HydroForce h = GetPropulsorForce();

        dragImpulseFx_ += h.drag.x()*dt;
        reactiveImpulseFx_ += h.reactive.x()*dt;

        totalImpulseFx_ += h.total.x()*dt;
        totalImpulseFy_ += h.total.y()*dt;
        totalImpulseFz_ += h.total.z()*dt;

        meanTimeS_ += dt;
    }

    if (lastCsvTimeS_<0.0 ||
        elapsedTimeS_-lastCsvTimeS_>=csvPeriodS_) {
        RecordSample();
        lastCsvTimeS_ = elapsedTimeS_;
    }

    if (lastConsoleTimeS_<0.0 ||
        elapsedTimeS_-lastConsoleTimeS_>=consolePeriodS_) {
        PrintTelemetry();
        lastConsoleTimeS_ = elapsedTimeS_;
    }
}
