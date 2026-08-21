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

constexpr sf::Scalar kRadToDeg =
    57.29577951308232;

constexpr sf::Scalar kCaudalK =
    0.50;       // Nm/rad

constexpr sf::Scalar kCaudalC =
    0.005;      // Nms/rad

}

class CaudalSpringActuator final :
    public sf::Actuator
{
public:
    CaudalSpringActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,
        unsigned int joint,
        sf::Scalar k,
        sf::Scalar c)
        : sf::Actuator(name),
          dynamics_(dynamics),
          joint_(joint),
          k_(k),
          c_(c)
    {}

    sf::ActuatorType getType() const override
    {
        return sf::ActuatorType::MOTOR;
    }

    void Update(sf::Scalar dt) override
    {
        sf::Actuator::Update(dt);

        if (!dynamics_) {
            valid_ = false;
            return;
        }

        btMultibodyLink::eFeatherstoneJointType
            pt = btMultibodyLink::eInvalid,
            vt = btMultibodyLink::eInvalid;

        dynamics_->getJointPosition(
            joint_,q_,pt);

        dynamics_->getJointVelocity(
            joint_,qdot_,vt);

        valid_ =
            pt == btMultibodyLink::eRevolute &&
            vt == btMultibodyLink::eRevolute &&
            std::isfinite(static_cast<double>(q_)) &&
            std::isfinite(static_cast<double>(qdot_));

        if (!valid_) {
            torque_ = 0.0;
            return;
        }

        torque_ = -k_*q_-c_*qdot_;
        dynamics_->DriveJoint(joint_,torque_);
    }

    sf::Scalar Position() const { return q_; }
    sf::Scalar Velocity() const { return qdot_; }
    sf::Scalar Torque() const { return torque_; }
    bool Valid() const { return valid_; }

private:
    sf::FeatherstoneEntity* dynamics_ = nullptr;
    unsigned int joint_ = 0;

    sf::Scalar k_ = 0.0;
    sf::Scalar c_ = 0.0;

    sf::Scalar q_ = 0.0;
    sf::Scalar qdot_ = 0.0;
    sf::Scalar torque_ = 0.0;

    bool valid_ = false;
};

TendonStageR2Simulator::TendonStageR2Simulator(
    sf::Scalar sps)
    : sf::SimulationManager(sps)
{
    setCallSimulationStepCompleted(true);
}

void TendonStageR2Simulator::BuildScenario()
{
    auto* app = sf::SimulationApp::getApp();

    if (!app)
        throw std::runtime_error(
            "SimulationApp unavailable.");

    const std::string scenario =
        app->getDataPath()+"env/static_pool.scn";

    std::cout << "\nLoading:\n  "
              << scenario << "\n";

    sf::ScenarioParser parser(this);

    if (!parser.Parse(scenario))
        throw std::runtime_error(
            "Failed to parse static_pool.scn");

    BindFish();
    SetNeutralInitialCondition();
    RegisterTendonActuator();
    RegisterCaudalSpringActuator();
    OpenCsv();
    ConfigureCamera();

    std::cout
        << "\n============================================================\n"
        << " BionicFish PHASE 0 - Thrust Validation\n"
        << "============================================================\n"
        << "Body fixed                 YES\n"
        << "Motor                      DISABLED\n"
        << "Tendon peak                3.0 N\n"
        << "Tendon frequency           0.60 Hz\n"
        << "Tail stiffness             0.65 Nm/rad\n"
        << "Caudal spring k            " << kCaudalK << " Nm/rad\n"
        << "Caudal damping c           " << kCaudalC << " Nms/rad\n"
        << "Tendon anchor reaction     ENABLED\n"
        << "Jacobian check             ENABLED\n"
        << "Load-cell reconstruction   ENABLED\n"
        << "Forward                    +X\n"
        << "CSV                        tethered_thrust_test.csv\n"
        << "============================================================\n\n";
}

void TendonStageR2Simulator::BindFish()
{
    fishRobot_ = getRobot("BionicFish");

    if (!fishRobot_)
        throw std::runtime_error(
            "BionicFish not found.");

    fishFeatherstoneRobot_ =
        dynamic_cast<sf::FeatherstoneRobot*>(
            fishRobot_);

    if (!fishFeatherstoneRobot_)
        throw std::runtime_error(
            "BionicFish is not FeatherstoneRobot.");

    fishDynamics_ =
        fishFeatherstoneRobot_->getDynamics();

    fishBody_ =
        fishRobot_->getBaseLink();

    if (!fishDynamics_ || !fishBody_)
        throw std::runtime_error(
            "Fish dynamics unavailable.");

    bodyLinkIndex_ = FindLinkIndex("Body");

    if (bodyLinkIndex_ != 0)
        throw std::runtime_error(
            "Phase 0 expects Body to be Featherstone base link.");

    for (std::size_t i=0; i<5; ++i) {
        const std::string j =
            "TailJoint"+std::to_string(i);

        const std::string l =
            "Tail"+std::to_string(i);

        tailJointIndices_[i] =
            FindJointIndex(j.c_str());

        tailLinkIndices_[i] =
            FindLinkIndex(l.c_str());

        if (tailJointIndices_[i] < 0 ||
            tailLinkIndices_[i] < 0)
            throw std::runtime_error(
                "Cannot bind "+j+"/"+l);

        tailSolids_[i] =
            fishDynamics_->getLink(
                static_cast<unsigned int>(
                    tailLinkIndices_[i])).solid;
    }

    caudalJointIndex_ =
        FindJointIndex("CaudalJoint");

    caudalLinkIndex_ =
        FindLinkIndex("CaudalFin");

    if (caudalJointIndex_ < 0 ||
        caudalLinkIndex_ < 0)
        throw std::runtime_error(
            "Caudal joint/link unavailable.");

    caudalFin_ =
        fishDynamics_->getLink(
            static_cast<unsigned int>(
                caudalLinkIndex_)).solid;

    std::cout << "\nPhase-0 binding:\n";

    for (unsigned int i=0;
         i<fishDynamics_->getNumOfJoints();
         ++i) {
        const auto j =
            fishDynamics_->getJoint(i);

        std::cout
            << "  joint " << i
            << " " << fishDynamics_->getJointName(i)
            << " parent=" << j.parent
            << " child=" << j.child;

        if (j.parent ==
            static_cast<unsigned int>(bodyLinkIndex_))
            std::cout << "  [BODY CHILD]";

        std::cout << "\n";
    }

    std::cout << std::endl;
}

int TendonStageR2Simulator::FindJointIndex(
    const char* name) const
{
    if (!fishDynamics_ || !name)
        return -1;

    const std::string a(name);
    const std::string b =
        "BionicFish/"+a;

    for (unsigned int i=0;
         i<fishDynamics_->getNumOfJoints();
         ++i) {
        const auto n =
            fishDynamics_->getJointName(i);

        if (n==a || n==b)
            return static_cast<int>(i);
    }

    return -1;
}

int TendonStageR2Simulator::FindLinkIndex(
    const char* name) const
{
    if (!fishDynamics_ || !name)
        return -1;

    const std::string a(name);
    const std::string b =
        "BionicFish/"+a;

    for (unsigned int i=0;
         i<fishDynamics_->getNumOfLinks();
         ++i) {
        auto* s =
            fishDynamics_->getLink(i).solid;

        if (!s)
            continue;

        const auto n = s->getName();

        if (n==a || n==b)
            return static_cast<int>(i);
    }

    return -1;
}

void TendonStageR2Simulator::SetNeutralInitialCondition()
{
    for (int j : tailJointIndices_)
        fishDynamics_->setJointIC(
            static_cast<unsigned int>(j),
            0.0,0.0);

    fishDynamics_->setJointIC(
        static_cast<unsigned int>(
            caudalJointIndex_),
        0.0,0.0);
}

void TendonStageR2Simulator::RegisterTendonActuator()
{
    TendonTailParameters p;

    p.passiveStiffnessNmRad = 0.65;
    p.passiveDampingNmsRad = 0.0;

    p.directTestTensionN = 3.0;
    p.directTestFrequencyHz = 0.60;
    p.startTimeS = 1.0;
    p.rampTimeS = 1.0;

    p.jointSafetyLimitRad =
        1.0471975511965976;

    p.jacobianEpsilonRad =
        1.0e-5;

    std::array<unsigned int,5> joints{};
    std::array<unsigned int,5> links{};

    for (std::size_t i=0; i<5; ++i) {
        joints[i] =
            static_cast<unsigned int>(
                tailJointIndices_[i]);

        links[i] =
            static_cast<unsigned int>(
                tailLinkIndices_[i]);
    }

    tendonActuator_ =
        new TendonTailActuator(
            "BionicFish/Phase0Tendon",
            fishDynamics_,
            static_cast<unsigned int>(
                bodyLinkIndex_),
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
    auto* tb = getTrackball();

    if (!tb || !fishBody_)
        return;

    tb->GlueToMoving(fishBody_);
    tb->UpdateCenterPos();
    tb->MouseScroll(-10.5f);
    tb->UpdateTransform();
}

sf::Vector3
TendonStageR2Simulator::GetSurfaceHydroForce(
    sf::SolidEntity* s) const
{
    if (!s)
        return sf::Vector3(0,0,0);

    sf::Vector3 Fb,Tb,Fd,Td,Ff,Tf;

    s->getHydrodynamicForces(
        Fb,Tb,Fd,Td,Ff,Tf);

    return Fd+Ff;
}

sf::Vector3
TendonStageR2Simulator::GetTailSurfaceForce() const
{
    sf::Vector3 F(0,0,0);

    for (auto* s : tailSolids_)
        F += GetSurfaceHydroForce(s);

    F += GetSurfaceHydroForce(caudalFin_);

    return F;
}

/*
 Fixed Body force balance:

 0 = F_body_external
     + sum(F_child_on_body)
     + R_support

 Stonefish getJointFeedback() returns
 parent-on-child reaction.

 Therefore:
 child-on-body = -F_joint

 and:
 R_support =
     -F_body_external
     + sum(F_joint_parent_on_child)

 Torque is reconstructed around Body CG.
*/
TendonStageR2Simulator::Wrench
TendonStageR2Simulator::GetSupportReaction() const
{
    Wrench R;

    if (!fishDynamics_ || !fishBody_)
        return R;

    const sf::Vector3 bodyCG =
        fishDynamics_->getLinkTransform(
            static_cast<unsigned int>(
                bodyLinkIndex_)).getOrigin();

    const sf::Vector3 bodyAppliedForce =
        fishBody_->getAppliedForce();

    const sf::Vector3 bodyAppliedTorque =
        fishDynamics_->getMultiBody()
            ->getBaseTorque();

    R.force = -bodyAppliedForce;
    R.torque = -bodyAppliedTorque;

    for (unsigned int i=0;
         i<fishDynamics_->getNumOfJoints();
         ++i) {
        const auto j =
            fishDynamics_->getJoint(i);

        if (j.parent !=
            static_cast<unsigned int>(
                bodyLinkIndex_))
            continue;

        sf::Vector3 fChild, tChild;

        fishDynamics_->getJointFeedback(
            i,fChild,tChild);

        const auto childT =
            fishDynamics_->getLinkTransform(
                j.child);

        // Feedback is expressed in child CG frame.
        const sf::Vector3 fWorld =
            childT.getBasis()*fChild;

        const sf::Vector3 tWorld =
            childT.getBasis()*tChild;

        const sf::Vector3 r =
            childT.getOrigin()-bodyCG;

        R.force += fWorld;

        R.torque +=
            tWorld+r.cross(fWorld);
    }

    return R;
}

void TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "tethered_thrust_test.csv",
        std::ios::out|std::ios::trunc);

    if (!csv_)
        throw std::runtime_error(
            "Cannot create CSV.");

    csv_
        << "time_s,phase,"
        << "left_tension_n,right_tension_n,"
        << "left_length_m,right_length_m,";

    for (int i=0; i<5; ++i)
        csv_
            << "j" << i << "_deg,"
            << "j" << i << "_qdot_rad_s,";

    csv_
        << "caudal_deg,"
        << "caudal_qdot_rad_s,"
        << "caudal_spring_torque_nm,"
        << "anchor_fx_n,anchor_fy_n,anchor_fz_n,"
        << "tendon_force_residual_n,"
        << "tendon_torque_residual_nm,";

    for (int i=0; i<5; ++i)
        csv_
            << "jt" << i
            << "_pointforce_nm,";

    for (int i=0; i<5; ++i)
        csv_
            << "jt" << i
            << "_jacobian_nm,";

    csv_
        << "jacobian_max_error_nm,";

    for (int i=0; i<5; ++i)
        csv_
            << "tail" << i
            << "_surface_fx_n,";

    csv_
        << "caudal_surface_fx_n,"
        << "tail_surface_fx_n,"
        << "tail_surface_fy_n,"
        << "tail_surface_fz_n,"
        << "support_rx_n,"
        << "support_ry_n,"
        << "support_rz_n,"
        << "support_tx_nm,"
        << "support_ty_nm,"
        << "support_tz_nm,"
        << "loadcell_thrust_fx_n,"
        << "mean_loadcell_thrust_fx_n,"
        << "mean_tail_surface_fx_n,"
        << "tendon_safety_tripped,"
        << "caudal_valid\n";

    csv_ << std::setprecision(10);
}

void TendonStageR2Simulator::RecordSample()
{
    if (!csv_ ||
        !tendonActuator_ ||
        !caudalSpringActuator_)
        return;

    const auto s =
        tendonActuator_->GetSnapshot();

    std::array<sf::Vector3,5> tailF;

    for (std::size_t i=0; i<5; ++i)
        tailF[i] =
            GetSurfaceHydroForce(
                tailSolids_[i]);

    const auto caudalF =
        GetSurfaceHydroForce(caudalFin_);

    const sf::Scalar meanThrust =
        meanAccumTimeS_ > 0.0
        ? thrustImpulseNs_/meanAccumTimeS_
        : 0.0;

    const sf::Scalar meanSurface =
        meanAccumTimeS_ > 0.0
        ? tailSurfaceImpulseNs_/meanAccumTimeS_
        : 0.0;

    csv_
        << s.timeS << ","
        << s.testPhase << ","
        << s.commandedTensionN[0] << ","
        << s.commandedTensionN[1] << ","
        << s.tendonLengthM[0] << ","
        << s.tendonLengthM[1] << ",";

    for (std::size_t i=0; i<5; ++i)
        csv_
            << s.jointPositionRad[i]
                *kRadToDeg << ","
            << s.jointVelocityRadS[i]
            << ",";

    csv_
        << caudalSpringActuator_->Position()
            *kRadToDeg << ","
        << caudalSpringActuator_->Velocity() << ","
        << caudalSpringActuator_->Torque() << ","

        << s.bodyAnchorForceWorld.x() << ","
        << s.bodyAnchorForceWorld.y() << ","
        << s.bodyAnchorForceWorld.z() << ","

        << s.tendonNetForceWorld.length() << ","
        << s.tendonNetTorqueWorld.length() << ",";

    for (auto v : s.tendonTorqueFromForcesNm)
        csv_ << v << ",";

    for (auto v : s.tendonTorqueFromJacobianNm)
        csv_ << v << ",";

    csv_ << s.jacobianMaxErrorNm << ",";

    for (const auto& F : tailF)
        csv_ << F.x() << ",";

    csv_
        << caudalF.x() << ","

        << lastTailSurface_.x() << ","
        << lastTailSurface_.y() << ","
        << lastTailSurface_.z() << ","

        << lastSupport_.force.x() << ","
        << lastSupport_.force.y() << ","
        << lastSupport_.force.z() << ","

        << lastSupport_.torque.x() << ","
        << lastSupport_.torque.y() << ","
        << lastSupport_.torque.z() << ","

        << lastLoadcellThrustFx_ << ","
        << meanThrust << ","
        << meanSurface << ","

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

    const sf::Scalar meanThrust =
        meanAccumTimeS_ > 0.0
        ? thrustImpulseNs_/meanAccumTimeS_
        : 0.0;

    std::cout
        << std::fixed
        << std::setprecision(4)

        << "[P0] t=" << s.timeS

        << " T=("
        << s.commandedTensionN[0]
        << ","
        << s.commandedTensionN[1]
        << ")"

        << " loadFx="
        << lastLoadcellThrustFx_

        << " meanFx="
        << meanThrust

        << " tailSurfFx="
        << lastTailSurface_.x()

        << " consF="
        << s.tendonNetForceWorld.length()

        << " consT="
        << s.tendonNetTorqueWorld.length()

        << " jacErr="
        << s.jacobianMaxErrorNm

        << " C="
        << caudalSpringActuator_->Position()
            *kRadToDeg
        << "deg"

        << " safety="
        << (s.safetyTripped ? "TRIPPED":"OK")

        << std::endl;
}

void TendonStageR2Simulator::SimulationStepCompleted(
    sf::Scalar dt)
{
    elapsedTimeS_ += dt;

    /*
     IMPORTANT:
     This runs after Bullet solved the step.
     Joint feedback therefore belongs to the
     step that just completed.
    */
    lastSupport_ =
        GetSupportReaction();

    lastTailSurface_ =
        GetTailSurfaceForce();

    /*
     R_support = stand force acting ON fish.

     Positive propulsion means fish would
     accelerate toward +X if released.

     Therefore the load-cell thrust sign is:
         F_thrust = -R_support
    */
    lastLoadcellThrustFx_ =
        -lastSupport_.force.x();

    if (elapsedTimeS_ >= meanStartTimeS_) {
        thrustImpulseNs_ +=
            lastLoadcellThrustFx_*dt;

        tailSurfaceImpulseNs_ +=
            lastTailSurface_.x()*dt;

        meanAccumTimeS_ += dt;
    }

    if (lastCsvTimeS_ < 0.0 ||
        elapsedTimeS_-lastCsvTimeS_
            >= csvPeriodS_) {
        RecordSample();
        lastCsvTimeS_ = elapsedTimeS_;
    }

    if (lastConsoleTimeS_ < 0.0 ||
        elapsedTimeS_-lastConsoleTimeS_
            >= consolePeriodS_) {
        PrintTelemetry();
        lastConsoleTimeS_ = elapsedTimeS_;
    }
}
