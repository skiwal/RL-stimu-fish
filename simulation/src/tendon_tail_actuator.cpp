#include "inc/tendon_tail_actuator.h"

#include <Stonefish/entities/SolidEntity.h>
#include <algorithm>
#include <cmath>

namespace {

constexpr sf::Scalar kTiny = 1.0e-9;
constexpr sf::Scalar kPi = 3.14159265358979323846;

constexpr std::array<sf::Scalar,5> kGuideX{
    0.0190, 0.0195, 0.0205, 0.0190, 0.0150
};

constexpr std::array<sf::Scalar,5> kGuideY{
    0.045, 0.035, 0.025, 0.015, 0.005
};

constexpr std::array<int,5> kRouting{0,0,0,1,1};

const sf::Vector3 kLeftAnchor(-0.0615,-0.055,-0.0345);
const sf::Vector3 kRightAnchor(-0.0615,0.055,0.0345);

sf::Vector3 RotateAroundAxis(
    const sf::Vector3& v,
    const sf::Vector3& axis,
    sf::Scalar a)
{
    const sf::Scalar c = std::cos(a);
    const sf::Scalar s = std::sin(a);

    return v*c
         + axis.cross(v)*s
         + axis*(axis.dot(v)*(1.0-c));
}

}

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
    if (!dynamics_ || bodyLinkIndex_ >= dynamics_->getNumOfLinks()) {
        safetyTripped_ = true;
        snapshot_.safetyTripped = true;
        return;
    }

    for (std::size_t s=0; s<2; ++s) {
        const auto path = BuildTendonPath(s==0);
        const auto L = ComputePathLength(path);

        initialTendonLengthM_[s] = L;
        snapshot_.initialTendonLengthM[s] = L;
        snapshot_.tendonLengthM[s] = L;
    }
}

sf::ActuatorType TendonTailActuator::getType() const
{
    return sf::ActuatorType::MOTOR;
}

bool TendonTailActuator::IsFiniteVector(const sf::Vector3& v)
{
    return std::isfinite(static_cast<double>(v.x()))
        && std::isfinite(static_cast<double>(v.y()))
        && std::isfinite(static_cast<double>(v.z()));
}

bool TendonTailActuator::ReadJointState(
    std::array<sf::Scalar,5>& q,
    std::array<sf::Scalar,5>& qDot) const
{
    if (!dynamics_)
        return false;

    for (std::size_t i=0; i<5; ++i) {
        btMultibodyLink::eFeatherstoneJointType pt, vt;

        dynamics_->getJointPosition(
            tailJointIndices_[i], q[i], pt);

        dynamics_->getJointVelocity(
            tailJointIndices_[i], qDot[i], vt);

        if (pt != btMultibodyLink::eRevolute ||
            vt != btMultibodyLink::eRevolute ||
            !std::isfinite(static_cast<double>(q[i])) ||
            !std::isfinite(static_cast<double>(qDot[i])))
            return false;
    }

    return true;
}

sf::Vector3 TendonTailActuator::LocalPointToWorld(
    unsigned int link,
    const sf::Vector3& local) const
{
    if (!dynamics_ || link >= dynamics_->getNumOfLinks())
        return sf::Vector3(0,0,0);

    auto solid = dynamics_->getLink(link).solid;

    return solid
        ? solid->getOTransform()*local
        : sf::Vector3(0,0,0);
}

TendonTailActuator::Path
TendonTailActuator::BuildTendonPath(bool left) const
{
    Path p;

    p[0].linkIndex = bodyLinkIndex_;
    p[0].world = LocalPointToWorld(
        bodyLinkIndex_,
        left ? kLeftAnchor : kRightAnchor);

    for (std::size_t i=0; i<5; ++i) {
        sf::Scalar sign = left ? -1.0 : 1.0;

        if (kRouting[i])
            sign = -sign;

        p[i+1].linkIndex = tailLinkIndices_[i];
        p[i+1].world = LocalPointToWorld(
            tailLinkIndices_[i],
            sf::Vector3(
                -kGuideX[i],
                sign*kGuideY[i],
                0.0));
    }

    return p;
}

sf::Scalar TendonTailActuator::ComputePathLength(const Path& p)
{
    sf::Scalar L = 0.0;

    for (std::size_t i=0; i+1<p.size(); ++i) {
        const auto d = p[i+1].world-p[i].world;
        const auto l = d.length();

        if (!std::isfinite(static_cast<double>(l)) || l <= kTiny)
            return 0.0;

        L += l;
    }

    return L;
}

sf::Scalar TendonTailActuator::SmoothStep01(sf::Scalar x)
{
    x = std::max(sf::Scalar(0), std::min(sf::Scalar(1),x));
    return x*x*(3.0-2.0*x);
}

std::array<sf::Scalar,2>
TendonTailActuator::ComputeDirectTestTensions(
    sf::Scalar t,
    unsigned int& phase) const
{
    std::array<sf::Scalar,2> T{0.0,0.0};

    if (t < parameters_.startTimeS) {
        phase = 0;
        return T;
    }

    sf::Scalar envelope = 1.0;

    if (parameters_.rampTimeS > 0.0 &&
        t < parameters_.startTimeS+parameters_.rampTimeS)
        envelope = SmoothStep01(
            (t-parameters_.startTimeS)/parameters_.rampTimeS);

    const sf::Scalar phi =
        2.0*kPi*
        parameters_.directTestFrequencyHz*
        (t-parameters_.startTimeS);

    const sf::Scalar s = std::sin(phi);
    const sf::Scalar A =
        envelope*parameters_.directTestTensionN;

    if (s >= 0.0) {
        T[1] = A*s;
        phase = 1;
    } else {
        T[0] = A*(-s);
        phase = 2;
    }

    return T;
}

bool TendonTailActuator::JointAxisPivot(
    std::size_t i,
    sf::Vector3& axis,
    sf::Vector3& pivot) const
{
    if (!dynamics_ || i >= 5)
        return false;

    const auto joint =
        dynamics_->getJoint(tailJointIndices_[i]);

    if (joint.child >= dynamics_->getNumOfLinks())
        return false;

    const auto T =
        dynamics_->getLinkTransform(joint.child);

    axis = T.getBasis()*joint.axisInChild;

    if (axis.length2() <= kTiny)
        return false;

    axis.normalize();
    pivot = T*joint.pivotInChild;

    return true;
}

sf::Scalar TendonTailActuator::PerturbedPathLength(
    const Path& path,
    std::size_t joint,
    sf::Scalar delta) const
{
    sf::Vector3 axis, pivot;

    if (!JointAxisPivot(joint,axis,pivot))
        return 0.0;

    Path p = path;

    // q_joint changes the entire downstream subtree.
    for (std::size_t k=joint+1; k<p.size(); ++k) {
        const sf::Vector3 r = p[k].world-pivot;
        p[k].world =
            pivot+RotateAroundAxis(r,axis,delta);
    }

    return ComputePathLength(p);
}

void TendonTailActuator::ValidateTendon(
    const Path& path,
    const std::array<sf::Vector3,6>& F,
    sf::Scalar tension)
{
    if (tension <= 0.0)
        return;

    const sf::Scalar eps =
        parameters_.jacobianEpsilonRad;

    for (std::size_t j=0; j<5; ++j) {
        sf::Vector3 axis, pivot;

        if (!JointAxisPivot(j,axis,pivot)) {
            safetyTripped_ = true;
            return;
        }

        sf::Vector3 moment(0,0,0);

        // Only forces on the child subtree contribute to Q_j.
        for (std::size_t k=j+1; k<path.size(); ++k)
            moment +=
                (path[k].world-pivot).cross(F[k]);

        const sf::Scalar qForce =
            axis.dot(moment);

        const sf::Scalar Lp =
            PerturbedPathLength(path,j,+eps);

        const sf::Scalar Lm =
            PerturbedPathLength(path,j,-eps);

        if (Lp <= 0.0 || Lm <= 0.0) {
            safetyTripped_ = true;
            return;
        }

        const sf::Scalar dLdq =
            (Lp-Lm)/(2.0*eps);

        const sf::Scalar qJac =
            -tension*dLdq;

        snapshot_.tendonTorqueFromForcesNm[j] += qForce;
        snapshot_.tendonTorqueFromJacobianNm[j] += qJac;
    }
}

void TendonTailActuator::ApplyPointForce(
    unsigned int link,
    const sf::Vector3& point,
    const sf::Vector3& force)
{
    if (!dynamics_ ||
        link >= dynamics_->getNumOfLinks() ||
        !IsFiniteVector(point) ||
        !IsFiniteVector(force)) {
        safetyTripped_ = true;
        return;
    }

    const sf::Vector3 cg =
        dynamics_->getLinkTransform(link).getOrigin();

    dynamics_->AddLinkForce(link,force);
    dynamics_->AddLinkTorque(
        link,
        (point-cg).cross(force));
}

void TendonTailActuator::ApplyTendonForces(
    const Path& path,
    sf::Scalar tension)
{
    if (tension <= 0.0)
        return;

    std::array<sf::Vector3,6> F;

    for (auto& f : F)
        f = sf::Vector3(0,0,0);

    for (std::size_t i=0; i+1<path.size(); ++i) {
        sf::Vector3 d =
            path[i+1].world-path[i].world;

        const sf::Scalar L = d.length();

        if (L <= kTiny ||
            !std::isfinite(static_cast<double>(L))) {
            safetyTripped_ = true;
            return;
        }

        d /= L;

        const sf::Vector3 f = tension*d;

        F[i] += f;
        F[i+1] -= f;
    }

    sf::Vector3 netF(0,0,0);
    sf::Vector3 netT(0,0,0);

    for (std::size_t i=0; i<F.size(); ++i) {
        netF += F[i];
        netT += path[i].world.cross(F[i]);
    }

    snapshot_.bodyAnchorForceWorld += F[0];
    snapshot_.tendonNetForceWorld += netF;
    snapshot_.tendonNetTorqueWorld += netT;

    ValidateTendon(path,F,tension);

    // CRITICAL PHASE-0 CHANGE:
    // Apply ALL nodes, including path[0] on Body.
    for (std::size_t i=0; i<path.size(); ++i) {
        ApplyPointForce(
            path[i].linkIndex,
            path[i].world,
            F[i]);

        if (safetyTripped_)
            return;
    }
}

void TendonTailActuator::Update(sf::Scalar dt)
{
    sf::Actuator::Update(dt);

    if (!dynamics_ || dt <= 0.0) {
        safetyTripped_ = true;
        snapshot_.safetyTripped = true;
        return;
    }

    elapsedTimeS_ += dt;
    snapshot_.timeS = elapsedTimeS_;

    snapshot_.bodyAnchorForceWorld =
        sf::Vector3(0,0,0);

    snapshot_.tendonNetForceWorld =
        sf::Vector3(0,0,0);

    snapshot_.tendonNetTorqueWorld =
        sf::Vector3(0,0,0);

    snapshot_.tendonTorqueFromForcesNm.fill(0.0);
    snapshot_.tendonTorqueFromJacobianNm.fill(0.0);
    snapshot_.tendonTorqueErrorNm.fill(0.0);
    snapshot_.jacobianMaxErrorNm = 0.0;

    std::array<sf::Scalar,5> q{}, qDot{};

    if (!ReadJointState(q,qDot))
        safetyTripped_ = true;

    for (std::size_t i=0; i<5; ++i) {
        snapshot_.jointPositionRad[i] = q[i];
        snapshot_.jointVelocityRadS[i] = qDot[i];

        if (std::abs(q[i]) >
            parameters_.jointSafetyLimitRad)
            safetyTripped_ = true;
    }

    std::array<Path,2> paths{
        BuildTendonPath(true),
        BuildTendonPath(false)
    };

    for (std::size_t s=0; s<2; ++s) {
        const sf::Scalar L =
            ComputePathLength(paths[s]);

        if (L <= 0.0)
            safetyTripped_ = true;

        snapshot_.tendonLengthM[s] = L;
        snapshot_.initialTendonLengthM[s] =
            initialTendonLengthM_[s];

        snapshot_.tendonLengthChangeM[s] =
            L-initialTendonLengthM_[s];
    }

    unsigned int phase = 0;

    auto tension =
        ComputeDirectTestTensions(
            elapsedTimeS_,phase);

    if (safetyTripped_)
        tension = {0.0,0.0};

    snapshot_.testPhase = phase;
    snapshot_.commandedTensionN = tension;

    if (!safetyTripped_) {
        ApplyTendonForces(paths[0],tension[0]);

        if (!safetyTripped_)
            ApplyTendonForces(paths[1],tension[1]);
    }

    for (std::size_t i=0; i<5; ++i) {
        const sf::Scalar tau =
            -parameters_.passiveStiffnessNmRad*q[i]
            -parameters_.passiveDampingNmsRad*qDot[i];

        dynamics_->DriveJoint(
            tailJointIndices_[i],tau);

        snapshot_.passiveTorqueNm[i] = tau;

        const sf::Scalar err =
            snapshot_.tendonTorqueFromForcesNm[i]
            -snapshot_.tendonTorqueFromJacobianNm[i];

        snapshot_.tendonTorqueErrorNm[i] = err;
        snapshot_.jacobianMaxErrorNm =
            std::max(
                snapshot_.jacobianMaxErrorNm,
                std::abs(err));
    }

    snapshot_.safetyTripped = safetyTripped_;
}

TendonTailActuator::Snapshot
TendonTailActuator::GetSnapshot() const
{
    return snapshot_;
}
