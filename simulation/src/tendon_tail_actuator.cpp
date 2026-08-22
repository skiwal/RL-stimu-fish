#include "inc/tendon_tail_actuator.h"

#include <Stonefish/entities/SolidEntity.h>

#include <algorithm>
#include <cmath>

namespace
{

constexpr sf::Scalar kTiny = 1.0e-9;

// Original fishsim guide geometry.
// Tail0 -> Tail4
constexpr std::array<sf::Scalar,5> kGuideX{
    0.0190,
    0.0195,
    0.0205,
    0.0190,
    0.0150
};

constexpr std::array<sf::Scalar,5> kGuideY{
    0.045,
    0.035,
    0.025,
    0.015,
    0.005
};

// Original routing:
//
// LEFT:
// Body-L -> T0-L -> T1-L -> T2-L -> T3-R -> T4-R
//
// RIGHT:
// Body-R -> T0-R -> T1-R -> T2-R -> T3-L -> T4-L
//
constexpr std::array<int,5> kRouting{
    0, 0, 0, 1, 1
};

// Fixed diagnostic anchors on Body.
// Motor/crank are deliberately bypassed.
const sf::Vector3 kLeftAnchor(
    -0.0615,
    -0.055,
    -0.0345);

const sf::Vector3 kRightAnchor(
    -0.0615,
     0.055,
     0.0345);

sf::Vector3 RotateAroundAxis(
    const sf::Vector3& v,
    const sf::Vector3& axis,
    sf::Scalar angle)
{
    const sf::Scalar c = std::cos(angle);
    const sf::Scalar s = std::sin(angle);

    return
        v*c
        + axis.cross(v)*s
        + axis*(axis.dot(v)*(1.0-c));
}

} // namespace


TendonTailActuator::TendonTailActuator(
    const std::string& name,
    sf::FeatherstoneEntity* dynamics,
    unsigned int bodyLinkIndex,
    const std::array<unsigned int,5>& tailLinkIndices,
    const std::array<unsigned int,5>& tailJointIndices,
    const TendonTailParameters& parameters)
    : sf::Actuator(name),
      dynamics_(dynamics),
      bodyLinkIndex_(bodyLinkIndex),
      tailLinkIndices_(tailLinkIndices),
      tailJointIndices_(tailJointIndices),
      parameters_(parameters)
{
    if (!dynamics_ ||
        bodyLinkIndex_ >= dynamics_->getNumOfLinks())
    {
        safetyTripped_ = true;
        snapshot_.safetyTripped = true;
        return;
    }

    for (std::size_t s=0; s<2; ++s)
    {
        const Path path =
            BuildTendonPath(s == 0);

        const sf::Scalar L =
            ComputePathLength(path);

        if (L <= 0.0)
        {
            safetyTripped_ = true;
            snapshot_.safetyTripped = true;
            return;
        }

        initialTendonLengthM_[s] = L;

        snapshot_.initialTendonLengthM[s] = L;
        snapshot_.tendonLengthM[s] = L;
    }
}


sf::ActuatorType
TendonTailActuator::getType() const
{
    // Stonefish has no native spatial-tendon actuator.
    return sf::ActuatorType::MOTOR;
}


bool
TendonTailActuator::IsFiniteVector(
    const sf::Vector3& v)
{
    return
        std::isfinite(static_cast<double>(v.x())) &&
        std::isfinite(static_cast<double>(v.y())) &&
        std::isfinite(static_cast<double>(v.z()));
}


bool
TendonTailActuator::ReadJointState(
    std::array<sf::Scalar,5>& q,
    std::array<sf::Scalar,5>& qDot) const
{
    if (!dynamics_)
        return false;

    for (std::size_t i=0; i<5; ++i)
    {
        btMultibodyLink::eFeatherstoneJointType
            positionType =
                btMultibodyLink::eInvalid;

        btMultibodyLink::eFeatherstoneJointType
            velocityType =
                btMultibodyLink::eInvalid;

        dynamics_->getJointPosition(
            tailJointIndices_[i],
            q[i],
            positionType);

        dynamics_->getJointVelocity(
            tailJointIndices_[i],
            qDot[i],
            velocityType);

        if (positionType != btMultibodyLink::eRevolute ||
            velocityType != btMultibodyLink::eRevolute ||
            !std::isfinite(static_cast<double>(q[i])) ||
            !std::isfinite(static_cast<double>(qDot[i])))
        {
            return false;
        }
    }

    return true;
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
        dynamics_->getLink(linkIndex).solid;

    if (!solid)
        return sf::Vector3(0,0,0);

    // Guide coordinates are defined relative to the
    // physical body origin, not the CG.
    return solid->getOTransform()*localPoint;
}


TendonTailActuator::Path
TendonTailActuator::BuildTendonPath(
    bool left) const
{
    Path path{};

    // P0 = fixed Body anchor.
    path[0].linkIndex =
        bodyLinkIndex_;

    path[0].world =
        LocalPointToWorld(
            bodyLinkIndex_,
            left ? kLeftAnchor : kRightAnchor);

    // P1..P5 = Tail0..Tail4 guides.
    for (std::size_t i=0; i<5; ++i)
    {
        sf::Scalar sideSign =
            left ? -1.0 : 1.0;

        if (kRouting[i] != 0)
            sideSign = -sideSign;

        const sf::Vector3 localGuide(
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
            path[i+1].world -
            path[i].world;

        const sf::Scalar L =
            d.length();

        if (L <= kTiny ||
            !std::isfinite(static_cast<double>(L)))
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
    if (!dynamics_ || i >= 5)
        return false;

    const auto joint =
        dynamics_->getJoint(
            tailJointIndices_[i]);

    if (joint.child >=
        dynamics_->getNumOfLinks())
    {
        return false;
    }

    const auto childTransform =
        dynamics_->getLinkTransform(
            joint.child);

    axisWorld =
        childTransform.getBasis() *
        joint.axisInChild;

    if (axisWorld.length2() <= kTiny)
        return false;

    axisWorld.normalize();

    pivotWorld =
        childTransform *
        joint.pivotInChild;

    return
        IsFiniteVector(axisWorld) &&
        IsFiniteVector(pivotWorld);
}


sf::Scalar
TendonTailActuator::PerturbedPathLength(
    const Path& original,
    std::size_t joint,
    sf::Scalar delta) const
{
    sf::Vector3 axis;
    sf::Vector3 pivot;

    if (!JointAxisPivot(
            joint,
            axis,
            pivot))
    {
        return 0.0;
    }

    Path perturbed =
        original;

    // Changing q_j rotates the complete child subtree:
    //
    // joint 0 -> Tail0..Tail4
    // joint 1 -> Tail1..Tail4
    // ...
    // joint 4 -> Tail4
    //
    // Path index:
    // 0 Body
    // 1 Tail0
    // ...
    // 5 Tail4
    for (std::size_t k=joint+1;
         k<perturbed.size();
         ++k)
    {
        const sf::Vector3 r =
            perturbed[k].world -
            pivot;

        perturbed[k].world =
            pivot +
            RotateAroundAxis(
                r,
                axis,
                delta);
    }

    return
        ComputePathLength(perturbed);
}


void
TendonTailActuator::ValidateTendon(
    const Path& path,
    const std::array<sf::Vector3,6>& forces,
    sf::Scalar tension)
{
    if (tension <= 0.0)
        return;

    const sf::Scalar eps =
        parameters_.jacobianEpsilonRad;

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

        // --------------------------------------------------------
        // A. Generalized torque produced by the ACTUAL point forces
        //    applied to the complete child subtree.
        // --------------------------------------------------------
        sf::Vector3 moment(0,0,0);

        for (std::size_t k=j+1;
             k<path.size();
             ++k)
        {
            moment +=
                (path[k].world-pivot)
                    .cross(forces[k]);
        }

        const sf::Scalar tauFromForces =
            axis.dot(moment);

        // --------------------------------------------------------
        // B. Independent virtual-work check:
        //
        //       Q_j = -T dL/dq_j
        // --------------------------------------------------------
        const sf::Scalar Lplus =
            PerturbedPathLength(
                path,
                j,
                +eps);

        const sf::Scalar Lminus =
            PerturbedPathLength(
                path,
                j,
                -eps);

        if (Lplus <= 0.0 ||
            Lminus <= 0.0)
        {
            safetyTripped_ = true;
            return;
        }

        const sf::Scalar dLdq =
            (Lplus-Lminus) /
            (2.0*eps);

        const sf::Scalar tauFromJacobian =
            -tension*dLdq;

        snapshot_
            .tendonTorqueFromForcesNm[j]
            += tauFromForces;

        snapshot_
            .tendonTorqueFromJacobianNm[j]
            += tauFromJacobian;
    }
}


void
TendonTailActuator::ApplyPointForce(
    unsigned int link,
    const sf::Vector3& point,
    const sf::Vector3& force)
{
    if (!dynamics_ ||
        link >= dynamics_->getNumOfLinks() ||
        !IsFiniteVector(point) ||
        !IsFiniteVector(force))
    {
        safetyTripped_ = true;
        return;
    }

    // Stonefish AddLinkForce applies at CG.
    // Therefore explicitly add the moment produced by
    // applying the cable force at the guide point.
    const sf::Vector3 cg =
        dynamics_
            ->getLinkTransform(link)
            .getOrigin();

    const sf::Vector3 torque =
        (point-cg).cross(force);

    dynamics_->AddLinkForce(
        link,
        force);

    dynamics_->AddLinkTorque(
        link,
        torque);
}


void
TendonTailActuator::ApplyTendonForces(
    const Path& path,
    sf::Scalar tension)
{
    if (tension <= 0.0)
        return;

    std::array<sf::Vector3,6> force{};

    for (auto& f : force)
        f = sf::Vector3(0,0,0);

    // Every cable segment pulls both endpoints
    // toward each other.
    //
    // Pi -------- Pj
    //
    // u = (Pj-Pi)/|Pj-Pi|
    //
    // Pi += +T u
    // Pj += -T u
    //
    // Therefore an intermediate guide naturally receives
    // the vector sum from its two adjacent cable segments.
    for (std::size_t i=0;
         i+1<path.size();
         ++i)
    {
        sf::Vector3 d =
            path[i+1].world -
            path[i].world;

        const sf::Scalar L =
            d.length();

        if (L <= kTiny ||
            !std::isfinite(static_cast<double>(L)))
        {
            safetyTripped_ = true;
            return;
        }

        d /= L;

        const sf::Vector3 segmentForce =
            tension*d;

        force[i] +=
            segmentForce;

        force[i+1] -=
            segmentForce;
    }

    // ------------------------------------------------------------
    // Store actual guide forces.
    // ------------------------------------------------------------
    snapshot_.bodyAnchorForceWorld +=
        force[0];

    for (std::size_t i=0; i<5; ++i)
    {
        snapshot_.guideForceWorld[i] +=
            force[i+1];
    }

    // ------------------------------------------------------------
    // Conservation check.
    // For a complete internal cable:
    //
    // ΣF ~= 0
    // Σ(r x F) ~= 0
    // ------------------------------------------------------------
    sf::Vector3 netForce(0,0,0);
    sf::Vector3 netTorque(0,0,0);

    for (std::size_t i=0;
         i<force.size();
         ++i)
    {
        netForce +=
            force[i];

        netTorque +=
            path[i].world
                .cross(force[i]);
    }

    snapshot_.tendonNetForceWorld +=
        netForce;

    snapshot_.tendonNetTorqueWorld +=
        netTorque;

    // Independent generalized-force validation.
    ValidateTendon(
        path,
        force,
        tension);

    if (safetyTripped_)
        return;

    // Apply ALL cable reactions, including the fixed
    // Body anchor. This keeps the virtual cable internally
    // force/torque conservative.
    for (std::size_t i=0;
         i<path.size();
         ++i)
    {
        ApplyPointForce(
            path[i].linkIndex,
            path[i].world,
            force[i]);

        if (safetyTripped_)
            return;
    }
}


void
TendonTailActuator::Update(
    sf::Scalar dt)
{
    sf::Actuator::Update(dt);

    if (!dynamics_ || dt <= 0.0)
    {
        safetyTripped_ = true;
        snapshot_.safetyTripped = true;
        return;
    }

    elapsedTimeS_ += dt;
    snapshot_.timeS = elapsedTimeS_;

    // Reset per-step diagnostics.
    snapshot_.bodyAnchorForceWorld =
        sf::Vector3(0,0,0);

    snapshot_.tendonNetForceWorld =
        sf::Vector3(0,0,0);

    snapshot_.tendonNetTorqueWorld =
        sf::Vector3(0,0,0);

    for (auto& f :
         snapshot_.guideForceWorld)
    {
        f = sf::Vector3(0,0,0);
    }

    snapshot_
        .tendonTorqueFromForcesNm
        .fill(0.0);

    snapshot_
        .tendonTorqueFromJacobianNm
        .fill(0.0);

    snapshot_
        .tendonTorqueErrorNm
        .fill(0.0);

    snapshot_.jacobianMaxErrorNm =
        0.0;

    // ------------------------------------------------------------
    // Joint states.
    // ------------------------------------------------------------
    std::array<sf::Scalar,5> q{};
    std::array<sf::Scalar,5> qDot{};

    if (!ReadJointState(q,qDot))
        safetyTripped_ = true;

    for (std::size_t i=0; i<5; ++i)
    {
        snapshot_.jointPositionRad[i] =
            q[i];

        snapshot_.jointVelocityRadS[i] =
            qDot[i];

        if (std::abs(q[i]) >
            parameters_.jointSafetyLimitRad)
        {
            safetyTripped_ = true;
        }
    }

    // ------------------------------------------------------------
    // Current spatial tendon geometry.
    // ------------------------------------------------------------
    const Path leftPath =
        BuildTendonPath(true);

    const Path rightPath =
        BuildTendonPath(false);

    const sf::Scalar leftLength =
        ComputePathLength(leftPath);

    const sf::Scalar rightLength =
        ComputePathLength(rightPath);

    if (leftLength <= 0.0 ||
        rightLength <= 0.0)
    {
        safetyTripped_ = true;
    }

    snapshot_.tendonLengthM[0] =
        leftLength;

    snapshot_.tendonLengthM[1] =
        rightLength;

    snapshot_.initialTendonLengthM[0] =
        initialTendonLengthM_[0];

    snapshot_.initialTendonLengthM[1] =
        initialTendonLengthM_[1];

    // ============================================================
    // THE ENTIRE DIAGNOSTIC:
    //
    // LEFT  = constant 1 N
    // RIGHT = 0 N
    //
    // No sine.
    // No alternating tendon.
    // No motor.
    // No crank.
    // No length-to-tension law.
    // ============================================================
    std::array<sf::Scalar,2> tension{
        parameters_.diagnosticLeftTensionN,
        0.0
    };

    if (safetyTripped_)
        tension = {0.0,0.0};

    snapshot_.commandedTensionN =
        tension;

    if (!safetyTripped_)
    {
        ApplyTendonForces(
            leftPath,
            tension[0]);
    }

    // ------------------------------------------------------------
    // Original passive hinges.
    // ------------------------------------------------------------
    for (std::size_t i=0; i<5; ++i)
    {
        const sf::Scalar passiveTorque =
            -parameters_.passiveStiffnessNmRad*q[i]
            -parameters_.passiveDampingNmsRad*qDot[i];

        dynamics_->DriveJoint(
            tailJointIndices_[i],
            passiveTorque);

        snapshot_.passiveTorqueNm[i] =
            passiveTorque;

        const sf::Scalar err =
            snapshot_
                .tendonTorqueFromForcesNm[i]
            -
            snapshot_
                .tendonTorqueFromJacobianNm[i];

        snapshot_
            .tendonTorqueErrorNm[i] =
            err;

        snapshot_.jacobianMaxErrorNm =
            std::max(
                snapshot_.jacobianMaxErrorNm,
                std::abs(err));
    }

    snapshot_.safetyTripped =
        safetyTripped_;
}


TendonTailActuator::Snapshot
TendonTailActuator::GetSnapshot() const
{
    return snapshot_;
}
