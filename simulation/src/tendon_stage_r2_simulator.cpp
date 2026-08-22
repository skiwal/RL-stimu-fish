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

namespace
{

constexpr sf::Scalar kRadToDeg =
    57.29577951308232;

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
        app->getDataPath() +
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
    RegisterTendonActuator();
    OpenCsv();
    ConfigureCamera();

    std::cout
        << "\n============================================================\n"
        << " S-BEND TENDON DIAGNOSTIC\n"
        << "============================================================\n"
        << "Body                  FIXED\n"
        << "CaudalFin             FIXED TO Tail4\n"
        << "Fluid model           WET / SUBMERGED\n"
        << "Left tendon           CONSTANT 1.0 N\n"
        << "Right tendon          0.0 N\n"
        << "Motor / crank         BYPASSED\n"
        << "Tail hinge k          0.65 Nm/rad\n"
        << "Tail hinge c          0.0 Nms/rad\n"
        << "Routing               [0,0,0,1,1]\n"
        << "Expected sign         J4 opposite to upstream joints\n"
        << "CSV                    s_bend_diagnostic.csv\n"
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

    if (bodyLinkIndex_ != 0)
    {
        throw std::runtime_error(
            "S-bend diagnostic expects Body as base link.");
    }

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        const std::string jointName =
            "TailJoint" +
            std::to_string(i);

        const std::string linkName =
            "Tail" +
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
                "Cannot bind " +
                jointName +
                " / " +
                linkName);
        }
    }

    std::cout
        << "\nS-bend binding:\n";

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
            << ", link="
            << fishDynamics_
                   ->getLink(
                       static_cast<unsigned int>(
                           tailLinkIndices_[i]))
                   .solid
                   ->getName()
            << "\n";
    }

    std::cout << std::endl;
}


int
TendonStageR2Simulator::FindJointIndex(
    const char* name) const
{
    if (!fishDynamics_ || !name)
        return -1;

    const std::string plain(name);

    const std::string scoped =
        "BionicFish/" +
        plain;

    for (unsigned int i=0;
         i<fishDynamics_
               ->getNumOfJoints();
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
    if (!fishDynamics_ || !name)
        return -1;

    const std::string plain(name);

    const std::string scoped =
        "BionicFish/" +
        plain;

    for (unsigned int i=0;
         i<fishDynamics_
               ->getNumOfLinks();
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
RegisterTendonActuator()
{
    TendonTailParameters p;

    p.passiveStiffnessNmRad =
        0.00;

    p.passiveDampingNmsRad =
        0.0;

    p.diagnosticLeftTensionN =
        4.0;

    p.jointSafetyLimitRad =
        1.3962634016;

    p.jacobianEpsilonRad =
        1.0e-5;

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
            "BionicFish/SBendDiagnosticTendon",
            fishDynamics_,
            static_cast<unsigned int>(
                bodyLinkIndex_),
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

    if (!tb || !fishBody_)
        return;

    tb->GlueToMoving(
        fishBody_);

    tb->UpdateCenterPos();
    tb->MouseScroll(-10.5f);
    tb->UpdateTransform();
}


void
TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "s_bend_diagnostic.csv",
        std::ios::out |
        std::ios::trunc);

    if (!csv_)
    {
        throw std::runtime_error(
            "Cannot create s_bend_diagnostic.csv");
    }

    csv_
        << "time_s,"
        << "left_tension_n,"
        << "right_tension_n,"
        << "left_length_m,"
        << "right_length_m,";

    for (int i=0; i<5; ++i)
    {
        csv_
            << "j"
            << i
            << "_deg,";
    }

    // Absolute / cumulative orientation of each tail link.
    for (int i=0; i<5; ++i)
    {
        csv_
            << "theta"
            << i
            << "_cum_deg,";
    }

    // Actual force acting at each LEFT tendon guide.
    for (int i=0; i<5; ++i)
    {
        csv_
            << "guide"
            << i
            << "_fx_n,"
            << "guide"
            << i
            << "_fy_n,"
            << "guide"
            << i
            << "_fz_n,";
    }

    // Generalized tendon torque from actual applied point forces.
    for (int i=0; i<5; ++i)
    {
        csv_
            << "jt"
            << i
            << "_force_nm,";
    }

    // Independent -T dL/dq calculation.
    for (int i=0; i<5; ++i)
    {
        csv_
            << "jt"
            << i
            << "_jacobian_nm,";
    }

    for (int i=0; i<5; ++i)
    {
        csv_
            << "jt"
            << i
            << "_error_nm,";
    }

    for (int i=0; i<5; ++i)
    {
        csv_
            << "jt"
            << i
            << "_passive_nm,";
    }

    csv_
        << "anchor_fx_n,"
        << "anchor_fy_n,"
        << "anchor_fz_n,"
        << "tendon_force_residual_n,"
        << "tendon_torque_residual_nm,"
        << "jacobian_max_error_nm,"
        << "safety_tripped\n";

    csv_
        << std::setprecision(12);
}


void
TendonStageR2Simulator::
RecordSample()
{
    if (!csv_ ||
        !tendonActuator_)
    {
        return;
    }

    const auto s =
        tendonActuator_
            ->GetSnapshot();

    csv_
        << s.timeS
        << ","
        << s.commandedTensionN[0]
        << ","
        << s.commandedTensionN[1]
        << ","
        << s.tendonLengthM[0]
        << ","
        << s.tendonLengthM[1]
        << ",";

    // Local hinge angles.
    for (std::size_t i=0;
         i<5;
         ++i)
    {
        csv_
            << s.jointPositionRad[i]
               * kRadToDeg
            << ",";
    }

    // Cumulative tail-link orientation.
    sf::Scalar cumulative = 0.0;

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        cumulative +=
            s.jointPositionRad[i];

        csv_
            << cumulative
               * kRadToDeg
            << ",";
    }

    // Guide point forces.
    for (const auto& F :
         s.guideForceWorld)
    {
        csv_
            << F.x()
            << ","
            << F.y()
            << ","
            << F.z()
            << ",";
    }

    for (sf::Scalar v :
         s.tendonTorqueFromForcesNm)
    {
        csv_
            << v
            << ",";
    }

    for (sf::Scalar v :
         s.tendonTorqueFromJacobianNm)
    {
        csv_
            << v
            << ",";
    }

    for (sf::Scalar v :
         s.tendonTorqueErrorNm)
    {
        csv_
            << v
            << ",";
    }

    for (sf::Scalar v :
         s.passiveTorqueNm)
    {
        csv_
            << v
            << ",";
    }

    csv_
        << s.bodyAnchorForceWorld.x()
        << ","
        << s.bodyAnchorForceWorld.y()
        << ","
        << s.bodyAnchorForceWorld.z()
        << ","
        << s.tendonNetForceWorld.length()
        << ","
        << s.tendonNetTorqueWorld.length()
        << ","
        << s.jacobianMaxErrorNm
        << ","
        << (s.safetyTripped ? 1 : 0)
        << "\n";
}


void
TendonStageR2Simulator::
PrintTelemetry()
{
    if (!tendonActuator_)
        return;

    const auto s =
        tendonActuator_
            ->GetSnapshot();

    std::cout
        << std::fixed
        << std::setprecision(4)

        << "[S-BEND] t="
        << s.timeS

        << " T=("
        << s.commandedTensionN[0]
        << ","
        << s.commandedTensionN[1]
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
        << ") tauF=(";

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        if (i)
            std::cout << ",";

        std::cout
            << s.tendonTorqueFromForcesNm[i];
    }

    std::cout
        << ") tauJ=(";

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        if (i)
            std::cout << ",";

        std::cout
            << s.tendonTorqueFromJacobianNm[i];
    }

    std::cout
        << ") guideFy=(";

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        if (i)
            std::cout << ",";

        std::cout
            << s.guideForceWorld[i].y();
    }

    std::cout
        << ") jacErr="
        << s.jacobianMaxErrorNm

        << " consF="
        << s.tendonNetForceWorld.length()

        << " consT="
        << s.tendonNetTorqueWorld.length()

        << " safety="
        << (s.safetyTripped
            ? "TRIPPED"
            : "OK")

        << std::endl;
}


void
TendonStageR2Simulator::
SimulationStepCompleted(
    sf::Scalar dt)
{
    elapsedTimeS_ += dt;

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
