#include "inc/tendon_tail_actuator.h"

#include <Stonefish/entities/SolidEntity.h>

#include <algorithm>
#include <cmath>


namespace
{

constexpr sf::Scalar kTiny =
    1.0e-9;


// ------------------------------------------------------------
// fishsim tendon guide locations.
// Tail0 -> Tail4
// ------------------------------------------------------------

constexpr std::array<sf::Scalar,5>
kGuideX{
    0.0190,
    0.0195,
    0.0205,
    0.0190,
    0.0150
};


constexpr std::array<sf::Scalar,5>
kGuideY{
    0.045,
    0.035,
    0.025,
    0.015,
    0.005
};


// ------------------------------------------------------------
// fishsim routing:
//
// LEFT:
// motor-L
// -> Tail0-L
// -> Tail1-L
// -> Tail2-L
// -> Tail3-R
// -> Tail4-R
//
// RIGHT:
// motor-R
// -> Tail0-R
// -> Tail1-R
// -> Tail2-R
// -> Tail3-L
// -> Tail4-L
// ------------------------------------------------------------

constexpr std::array<int,5>
kRouting{
    0,0,0,1,1
};


// ------------------------------------------------------------
// These are MOTOR-SHAFT local coordinates.
//
// Source fishsim:
//
// motorShaftLength = 0.120
// motorArmLength   = 0.0395
//
// bodyLeft:
// y = -0.060 + 0.005 = -0.055
// z = -0.0395 + 0.005 = -0.0345
//
// bodyRight:
// y = +0.055
// z = +0.0345
//
// X=0 because M1Joint itself is already positioned at
// Body x = -0.0615 m.
// ------------------------------------------------------------

const sf::Vector3 kLeftMotorSite(
    0.0,
    -0.055,
    -0.0345);


const sf::Vector3 kRightMotorSite(
    0.0,
    +0.055,
    +0.0345);

} // namespace



TendonTailActuator::
TendonTailActuator(
    const std::string& name,
    sf::FeatherstoneEntity* dynamics,

    unsigned int motorLinkIndex,
    unsigned int motorJointIndex,

    const std::array<unsigned int,5>& tailLinkIndices,
    const std::array<unsigned int,5>& tailJointIndices,

    const TendonTailParameters& parameters)

    : sf::Actuator(name),
      dynamics_(dynamics),
      motorLinkIndex_(motorLinkIndex),
      motorJointIndex_(motorJointIndex),
      tailLinkIndices_(tailLinkIndices),
      tailJointIndices_(tailJointIndices),
      parameters_(parameters)
{
    if (!dynamics_ ||
        motorLinkIndex_ >= dynamics_->getNumOfLinks())
    {
        safetyTripped_ = true;
        snapshot_.safetyTripped = true;
        return;
    }

    // --------------------------------------------------------
    // MuJoCo default springlength=-1 means:
    // use tendon length in reference configuration.
    //
    // Therefore our initial neutral geometry is L0.
    // --------------------------------------------------------

    const Path left =
        BuildTendonPath(true);

    const Path right =
        BuildTendonPath(false);

    restLengthM_[0] =
        ComputePathLength(left);

    restLengthM_[1] =
        ComputePathLength(right);

    if (restLengthM_[0] <= 0.0 ||
        restLengthM_[1] <= 0.0)
    {
        safetyTripped_ = true;
        snapshot_.safetyTripped = true;
        return;
    }

    previousLengthM_ =
        restLengthM_;

    snapshot_.restLengthM =
        restLengthM_;

    snapshot_.tendonLengthM =
        restLengthM_;
}



sf::ActuatorType
TendonTailActuator::getType() const
{
    // Custom spatial-tendon actuator.
    return sf::ActuatorType::MOTOR;
}



bool
TendonTailActuator::IsFiniteVector(
    const sf::Vector3& v)
{
    return
        std::isfinite(
            static_cast<double>(v.x()))
        &&
        std::isfinite(
            static_cast<double>(v.y()))
        &&
        std::isfinite(
            static_cast<double>(v.z()));
}



bool
TendonTailActuator::ReadRevoluteJoint(
    unsigned int jointIndex,
    sf::Scalar& q,
    sf::Scalar& qDot) const
{
    if (!dynamics_ ||
        jointIndex >= dynamics_->getNumOfJoints())
    {
        return false;
    }

    btMultibodyLink::eFeatherstoneJointType
        pType =
            btMultibodyLink::eInvalid;

    btMultibodyLink::eFeatherstoneJointType
        vType =
            btMultibodyLink::eInvalid;

    dynamics_->getJointPosition(
        jointIndex,
        q,
        pType);

    dynamics_->getJointVelocity(
        jointIndex,
        qDot,
        vType);

    return
        pType == btMultibodyLink::eRevolute
        &&
        vType == btMultibodyLink::eRevolute
        &&
        std::isfinite(static_cast<double>(q))
        &&
        std::isfinite(static_cast<double>(qDot));
}



sf::Vector3
TendonTailActuator::LocalPointToWorld(
    unsigned int linkIndex,
    const sf::Vector3& localPoint) const
{
    if (!dynamics_ ||
        linkIndex >= dynamics_->getNumOfLinks())
    {
        return sf::Vector3(0,0,0);
    }

    auto* solid =
        dynamics_
            ->getLink(linkIndex)
            .solid;

    if (!solid)
        return sf::Vector3(0,0,0);

    // Local coordinates are relative to the
    // physical link origin, not CG.
    return
        solid->getOTransform()
        *
        localPoint;
}



TendonTailActuator::Path
TendonTailActuator::BuildTendonPath(
    bool left) const
{
    Path path{};

    // --------------------------------------------------------
    // P0 = actual moving MotorShaft crank site.
    // --------------------------------------------------------

    path[0].linkIndex =
        motorLinkIndex_;

    path[0].world =
        LocalPointToWorld(
            motorLinkIndex_,
            left
                ? kLeftMotorSite
                : kRightMotorSite);


    // --------------------------------------------------------
    // P1..P5 = Tail0..Tail4 via points.
    // --------------------------------------------------------

    for (std::size_t i=0; i<5; ++i)
    {
        sf::Scalar sideSign =
            left
                ? -1.0
                : +1.0;

        if (kRouting[i] != 0)
            sideSign =
                -sideSign;

        const sf::Vector3
            localGuide(
                -kGuideX[i],
                sideSign*kGuideY[i],
                0.0);

        path[i+1].linkIndex =
            tailLinkIndices_[i];

        path[i+1].world =
            LocalPointToWorld(
                tailLinkIndices_[i],
                localGuide);
    }

    return path;
}



sf::Scalar
TendonTailActuator::ComputePathLength(
    const Path& path)
{
    sf::Scalar total = 0.0;

    for (std::size_t i=0;
         i+1<path.size();
         ++i)
    {
        const sf::Vector3 d =
            path[i+1].world
            -
            path[i].world;

        const sf::Scalar L =
            d.length();

        if (L <= kTiny ||
            !std::isfinite(
                static_cast<double>(L)))
        {
            return 0.0;
        }

        total += L;
    }

    return total;
}



bool
TendonTailActuator::JointAxisPivot(
    std::size_t i,
    sf::Vector3& axisWorld,
    sf::Vector3& pivotWorld) const
{
    if (!dynamics_ ||
        i >= 5)
    {
        return false;
    }

    const auto joint =
        dynamics_->getJoint(
            tailJointIndices_[i]);

    if (joint.child >=
        dynamics_->getNumOfLinks())
    {
        return false;
    }

    const auto T =
        dynamics_->getLinkTransform(
            joint.child);

    axisWorld =
        T.getBasis()
        *
        joint.axisInChild;

    if (axisWorld.length2() <= kTiny)
        return false;

    axisWorld.normalize();

    // --------------------------------------------------------
    // Correct Stonefish Featherstone pivot relation:
    //
    // pivotInChild =
    //     R^T * (childCOM - jointPivot)
    //
    // therefore:
    //
    // jointPivot =
    //     childCOM - R*pivotInChild
    // --------------------------------------------------------

    pivotWorld =
        T.getOrigin()
        -
        T.getBasis()
        *
        joint.pivotInChild;

    return
        IsFiniteVector(axisWorld)
        &&
        IsFiniteVector(pivotWorld);
}



void
TendonTailActuator::ApplyPointForce(
    unsigned int linkIndex,
    const sf::Vector3& point,
    const sf::Vector3& force)
{
    if (!dynamics_ ||
        linkIndex >= dynamics_->getNumOfLinks() ||
        !IsFiniteVector(point) ||
        !IsFiniteVector(force))
    {
        safetyTripped_ = true;
        return;
    }

    // Stonefish AddLinkForce acts at CG.
    // Explicitly add r x F for force at the tendon guide.
    const sf::Vector3 cg =
        dynamics_
            ->getLinkTransform(linkIndex)
            .getOrigin();

    const sf::Vector3 torque =
        (point-cg)
            .cross(force);

    dynamics_->AddLinkForce(
        linkIndex,
        force);

    dynamics_->AddLinkTorque(
        linkIndex,
        torque);
}



void
TendonTailActuator::ApplySignedTendon(
    const Path& path,
    sf::Scalar signedForceN,
    std::size_t side)
{
    if (side >= 2 ||
        std::abs(signedForceN) <= kTiny)
    {
        return;
    }

    std::array<sf::Vector3,6>
        nodeForce{};

    for (auto& f : nodeForce)
        f = sf::Vector3(0,0,0);


    // --------------------------------------------------------
    // Every tendon segment:
    //
    // u = (Pj-Pi)/|Pj-Pi|
    //
    // positive signedForce:
    //     pulling/tension
    //
    // negative signedForce:
    //     MuJoCo linear-spring compression side
    //
    // This is deliberately SOURCE-FAITHFUL.
    // --------------------------------------------------------

    for (std::size_t i=0;
         i+1<path.size();
         ++i)
    {
        sf::Vector3 d =
            path[i+1].world
            -
            path[i].world;

        const sf::Scalar L =
            d.length();

        if (L <= kTiny ||
            !std::isfinite(
                static_cast<double>(L)))
        {
            safetyTripped_ = true;
            return;
        }

        d /= L;

        const sf::Vector3
            segmentForce =
                signedForceN*d;

        nodeForce[i] +=
            segmentForce;

        nodeForce[i+1] -=
            segmentForce;
    }


    // --------------------------------------------------------
    // Store guide forces.
    // --------------------------------------------------------

    snapshot_
        .motorAnchorForceWorld[side]
        =
        nodeForce[0];

    for (std::size_t i=0; i<5; ++i)
    {
        snapshot_
            .guideForceWorld[side][i]
            =
            nodeForce[i+1];
    }


    // --------------------------------------------------------
    // Generalized tendon torque on TailJoint0..4,
    // computed from actual point forces.
    // --------------------------------------------------------

    for (std::size_t j=0; j<5; ++j)
    {
        sf::Vector3 axis;
        sf::Vector3 pivot;

        if (!JointAxisPivot(
                j,
                axis,
                pivot))
        {
            safetyTripped_ = true;
            return;
        }

        sf::Vector3 moment(0,0,0);

        // joint j rotates Tail_j and all downstream links.
        for (std::size_t k=j+1;
             k<path.size();
             ++k)
        {
            moment +=
                (path[k].world-pivot)
                    .cross(nodeForce[k]);
        }

        const sf::Scalar tau =
            axis.dot(moment);

        snapshot_
            .tendonTorqueNm[side][j]
            =
            tau;

        snapshot_
            .totalTendonTorqueNm[j]
            +=
            tau;
    }


    // --------------------------------------------------------
    // Internal-force conservation check.
    // --------------------------------------------------------

    sf::Vector3 netF(0,0,0);
    sf::Vector3 netT(0,0,0);

    for (std::size_t i=0;
         i<nodeForce.size();
         ++i)
    {
        netF +=
            nodeForce[i];

        netT +=
            path[i].world
                .cross(nodeForce[i]);
    }

    snapshot_
        .tendonNetForceResidualWorld
        += netF;

    snapshot_
        .tendonNetTorqueResidualWorld
        += netT;


    // --------------------------------------------------------
    // Apply ALL reactions:
    //
    // importantly P0 force goes to the REAL MotorShaft.
    //
    // Thus tendon load feeds back into motor torque.
    // --------------------------------------------------------

    for (std::size_t i=0;
         i<path.size();
         ++i)
    {
        ApplyPointForce(
            path[i].linkIndex,
            path[i].world,
            nodeForce[i]);

        if (safetyTripped_)
            return;
    }
}



void
TendonTailActuator::Update(
    sf::Scalar dt)
{
    sf::Actuator::Update(dt);

    if (!dynamics_ ||
        dt <= 0.0)
    {
        safetyTripped_ = true;
        snapshot_.safetyTripped = true;
        return;
    }

    elapsedTimeS_ += dt;

    snapshot_.timeS =
        elapsedTimeS_;


    // ========================================================
    // RESET PER-STEP DATA
    // ========================================================

    snapshot_
        .tendonNetForceResidualWorld =
        sf::Vector3(0,0,0);

    snapshot_
        .tendonNetTorqueResidualWorld =
        sf::Vector3(0,0,0);

    snapshot_
        .totalTendonTorqueNm
        .fill(0.0);

    for (std::size_t side=0;
         side<2;
         ++side)
    {
        snapshot_
            .tendonTorqueNm[side]
            .fill(0.0);

        snapshot_
            .motorAnchorForceWorld[side]
            =
            sf::Vector3(0,0,0);

        for (auto& f :
             snapshot_
                 .guideForceWorld[side])
        {
            f =
                sf::Vector3(0,0,0);
        }
    }


    // ========================================================
    // MOTOR STATE
    // ========================================================

    sf::Scalar motorQ = 0.0;
    sf::Scalar motorQDot = 0.0;

    if (!ReadRevoluteJoint(
            motorJointIndex_,
            motorQ,
            motorQDot))
    {
        safetyTripped_ = true;
    }

    snapshot_.motorPositionRad =
        motorQ;

    snapshot_.motorVelocityRadS =
        motorQDot;


    // ========================================================
    // TAIL JOINT STATE
    // ========================================================

    std::array<sf::Scalar,5> q{};
    std::array<sf::Scalar,5> qDot{};

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        if (!ReadRevoluteJoint(
                tailJointIndices_[i],
                q[i],
                qDot[i]))
        {
            safetyTripped_ = true;
        }

        snapshot_
            .jointPositionRad[i]
            =
            q[i];

        snapshot_
            .jointVelocityRadS[i]
            =
            qDot[i];

        if (std::abs(q[i]) >
            parameters_
                .jointSafetyLimitRad)
        {
            safetyTripped_ = true;
        }
    }


    // ========================================================
    // CURRENT DUAL TENDON GEOMETRY
    // ========================================================

    const Path pathLeft =
        BuildTendonPath(true);

    const Path pathRight =
        BuildTendonPath(false);

    const std::array<Path,2>
        paths{
            pathLeft,
            pathRight
        };


    for (std::size_t side=0;
         side<2;
         ++side)
    {
        const sf::Scalar L =
            ComputePathLength(
                paths[side]);

        if (L <= 0.0 ||
            !std::isfinite(
                static_cast<double>(L)))
        {
            safetyTripped_ = true;
            continue;
        }

        snapshot_
            .tendonLengthM[side]
            =
            L;

        snapshot_
            .restLengthM[side]
            =
            restLengthM_[side];

        snapshot_
            .tendonDeltaLengthM[side]
            =
            L-restLengthM_[side];


        // ----------------------------------------------------
        // MuJoCo has direct tendon velocity from Jacobian.
        // Here we use high-rate finite difference.
        // ----------------------------------------------------

        sf::Scalar Ldot = 0.0;

        if (lengthHistoryReady_)
        {
            Ldot =
                (L-previousLengthM_[side])
                /
                dt;
        }

        snapshot_
            .tendonLengthVelocityMS[side]
            =
            Ldot;


        // ----------------------------------------------------
        // Source-faithful MuJoCo-style linear tendon:
        //
        // F = k*(L-L0) + c*dL/dt
        //
        // Positive -> pull.
        // Negative -> source-model compression/push.
        // ----------------------------------------------------

        const sf::Scalar F =
            parameters_
                .tendonStiffnessNPerM
            *
            snapshot_
                .tendonDeltaLengthM[side]
            +
            parameters_
                .tendonDampingNsPerM
            *
            Ldot;

        snapshot_
            .tendonForceN[side]
            =
            F;

        if (!std::isfinite(
                static_cast<double>(F))
            ||
            std::abs(F) >
                parameters_
                    .tendonForceSafetyLimitN)
        {
            safetyTripped_ = true;
        }
    }


    // ========================================================
    // APPLY LEFT + RIGHT TENDONS
    // ========================================================

    if (!safetyTripped_)
    {
        ApplySignedTendon(
            pathLeft,
            snapshot_.tendonForceN[0],
            0);

        ApplySignedTendon(
            pathRight,
            snapshot_.tendonForceN[1],
            1);
    }


    // ========================================================
    // PASSIVE NITINOL-EQUIVALENT HINGES
    // ========================================================

    for (std::size_t i=0;
         i<5;
         ++i)
    {
        const sf::Scalar passive =
            -parameters_
                .passiveStiffnessNmRad
                * q[i]
            -
            parameters_
                .passiveDampingNmsRad
                * qDot[i];

        dynamics_->DriveJoint(
            tailJointIndices_[i],
            passive);

        snapshot_
            .passiveTorqueNm[i]
            =
            passive;
    }


    // ========================================================
    // UPDATE LENGTH HISTORY
    // ========================================================

    previousLengthM_ =
        snapshot_.tendonLengthM;

    lengthHistoryReady_ =
        true;

    snapshot_.safetyTripped =
        safetyTripped_;
}



TendonTailActuator::Snapshot
TendonTailActuator::GetSnapshot() const
{
    return snapshot_;
}
