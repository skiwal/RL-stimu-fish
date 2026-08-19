#include "inc/tendon_tail_actuator.h"

#include <Stonefish/entities/SolidEntity.h>

#include <algorithm>
#include <cmath>

namespace
{

    constexpr sf::Scalar kTinyLength =
        1.0e-9;

    // ================================================================
    // Tail tendon guide geometry
    //
    // Coordinates are expressed in the BODY-ORIGIN frame of each
    // Tail link.
    //
    // local X negative = tailward
    // local Y          = lateral
    //
    // ================================================================

    constexpr std::array<sf::Scalar, 5>
        kGuideTailwardDistance =
            {
                0.0190,
                0.0195,
                0.0205,
                0.0190,
                0.0150};

    constexpr std::array<sf::Scalar, 5>
        kGuideLateralOffset =
            {
                0.045,
                0.035,
                0.025,
                0.015,
                0.005};

    // ================================================================
    // Reference routing
    //
    // 0 = stay on nominal side
    // 1 = cross to opposite side
    //
    // Left:
    //
    //   Tail0 L
    //   Tail1 L
    //   Tail2 L
    //   Tail3 R
    //   Tail4 R
    //
    // Right:
    //
    //   Tail0 R
    //   Tail1 R
    //   Tail2 R
    //   Tail3 L
    //   Tail4 L
    //
    // ================================================================

    constexpr std::array<int, 5>
        kRouting =
            {
                0,
                0,
                0,
                1,
                1};

    // ================================================================
    // FIXED tendon anchors for this diagnostic.
    //
    // IMPORTANT:
    //
    // There is NO motor motion in this experiment.
    //
    // M1 pivot in Body frame:
    //
    //     x = -0.0615 m
    //
    // Neutral left/right tendon sites:
    //
    //     left:
    //         y = -0.055
    //         z = -0.0345
    //
    //     right:
    //         y = +0.055
    //         z = +0.0345
    //
    // Therefore P0 is fixed to Body.
    // ================================================================

    const sf::Vector3
        kLeftFixedAnchorBodyLocal(
            -0.0615,
            -0.055,
            -0.0345);

    const sf::Vector3
        kRightFixedAnchorBodyLocal(
            -0.0615,
            0.055,
            0.0345);

} // namespace

TendonTailActuator::TendonTailActuator(
    const std::string &name,
    sf::FeatherstoneEntity *dynamics,
    unsigned int bodyLinkIndex,
    const std::array<unsigned int, 5> &tailLinkIndices,
    const std::array<unsigned int, 5> &tailJointIndices,
    const TendonTailParameters &parameters)
    : sf::Actuator(
          name),
      dynamics_(
          dynamics),
      bodyLinkIndex_(
          bodyLinkIndex),
      tailLinkIndices_(
          tailLinkIndices),
      tailJointIndices_(
          tailJointIndices),
      parameters_(
          parameters)
{
    if (
        dynamics_ == nullptr ||
        bodyLinkIndex_ >=
            dynamics_->getNumOfLinks())
    {
        safetyTripped_ =
            true;

        snapshot_.safetyTripped =
            true;

        return;
    }

    // ============================================================
    // Save neutral path lengths.
    //
    // They are diagnostic only.
    //
    // They DO NOT generate tension in this experiment.
    // ============================================================

    for (
        std::size_t side = 0;
        side < 2;
        ++side)
    {
        const bool
            leftTendon =
                side == 0;

        const auto
            path =
                BuildTendonPath(
                    leftTendon);

        const sf::Scalar
            length =
                ComputePathLength(
                    path);

        initialTendonLengthM_[side] =
            length;

        snapshot_.initialTendonLengthM[side] =
            length;

        snapshot_.tendonLengthM[side] =
            length;

        snapshot_.tendonLengthChangeM[side] =
            0.0;
    }
}

sf::ActuatorType
TendonTailActuator::getType() const
{
    // Stonefish does not have a native tendon actuator type.
    return sf::ActuatorType::MOTOR;
}

bool TendonTailActuator::ReadJointState(
    std::array<sf::Scalar, 5> &q,
    std::array<sf::Scalar, 5> &qDot) const
{
    if (
        dynamics_ == nullptr)
    {
        return false;
    }

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
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

        if (
            positionType !=
                btMultibodyLink::eRevolute ||
            velocityType !=
                btMultibodyLink::eRevolute ||
            !std::isfinite(
                static_cast<double>(
                    q[i])) ||
            !std::isfinite(
                static_cast<double>(
                    qDot[i])))
        {
            return false;
        }
    }

    return true;
}

sf::Vector3
TendonTailActuator::LocalPointToWorld(
    unsigned int linkIndex,
    const sf::Vector3 &localPoint) const
{
    if (
        dynamics_ == nullptr ||
        linkIndex >=
            dynamics_->getNumOfLinks())
    {
        return sf::Vector3(
            0.0,
            0.0,
            0.0);
    }

    const sf::FeatherstoneLink
        link =
            dynamics_->getLink(
                linkIndex);

    if (
        link.solid == nullptr)
    {
        return sf::Vector3(
            0.0,
            0.0,
            0.0);
    }

    // Guide coordinates are defined in body-origin coordinates.
    //
    // Therefore use getOTransform(), not CG transform.

    return link.solid->getOTransform() *
           localPoint;
}

std::array<TendonTailActuator::PathPoint, 6>
TendonTailActuator::BuildTendonPath(
    bool leftTendon) const
{
    std::array<PathPoint, 6>
        path;

    // ============================================================
    // P0
    //
    // FIXED Body tendon anchor.
    //
    // No MotorShaft.
    // No Servo.
    // No crank rotation.
    // ============================================================

    path[0].linkIndex =
        bodyLinkIndex_;

    path[0].world =
        LocalPointToWorld(
            bodyLinkIndex_,
            leftTendon
                ? kLeftFixedAnchorBodyLocal
                : kRightFixedAnchorBodyLocal);

    // ============================================================
    // P1..P5
    //
    // Tail0..Tail4 guide points.
    // ============================================================

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        sf::Scalar
            sideSign =
                leftTendon
                    ? -1.0
                    : 1.0;

        if (
            kRouting[i] != 0)
        {
            sideSign =
                -sideSign;
        }

        const sf::Vector3
            localGuide(
                -kGuideTailwardDistance[i],
                sideSign *
                    kGuideLateralOffset[i],
                0.0);

        path[i + 1].linkIndex =
            tailLinkIndices_[i];

        path[i + 1].world =
            LocalPointToWorld(
                tailLinkIndices_[i],
                localGuide);
    }

    return path;
}

sf::Scalar
TendonTailActuator::ComputePathLength(
    const std::array<PathPoint, 6> &path)
{
    sf::Scalar
        totalLength =
            0.0;

    for (
        std::size_t i = 0;
        i + 1 < path.size();
        ++i)
    {
        const sf::Vector3
            segment =
                path[i + 1].world -
                path[i].world;

        const sf::Scalar
            segmentLength =
                segment.length();

        if (
            !std::isfinite(
                static_cast<double>(
                    segmentLength)))
        {
            return 0.0;
        }

        totalLength +=
            segmentLength;
    }

    return totalLength;
}

sf::Scalar
TendonTailActuator::SmoothStep01(
    sf::Scalar x)
{
    x =
        std::max(
            sf::Scalar(0.0),
            std::min(
                sf::Scalar(1.0),
                x));

    return x *
           x *
           (3.0 -
            2.0 *
                x);
}

std::array<sf::Scalar, 2>
TendonTailActuator::ComputeDirectTestTensions(
    sf::Scalar timeS,
    unsigned int& phase) const
{
    std::array<sf::Scalar, 2>
        tension =
        {
            0.0,
            0.0
        };


    // ============================================================
    // Continuous antagonistic tendon test
    //
    // No motor.
    // No crank.
    // No length -> tension conversion.
    //
    // RIGHT:
    //
    //     T_R = A * max(0, sin(phi))
    //
    // LEFT:
    //
    //     T_L = A * max(0, -sin(phi))
    //
    // ============================================================

    constexpr sf::Scalar kPi =
        3.1415926535897932384626433832795;


    constexpr sf::Scalar
        startTimeS =
            1.0;


    constexpr sf::Scalar
        rampTimeS =
            1.0;


    constexpr sf::Scalar
        frequencyHz =
            0.6;


    constexpr sf::Scalar
        amplitudeN =
            3.0;


    // ------------------------------------------------------------
    // Initial neutral period
    // ------------------------------------------------------------

    if (
        timeS < startTimeS)
    {
        phase = 0;

        return tension;
    }


    // ------------------------------------------------------------
    // Smooth amplitude ramp.
    //
    // Avoid suddenly applying several Newtons at t = 1 s.
    // ------------------------------------------------------------

    sf::Scalar
        envelope =
            1.0;


    if (
        timeS
        <
        startTimeS
        +
        rampTimeS)
    {
        const sf::Scalar
            u =
                (
                    timeS
                    -
                    startTimeS
                )
                /
                rampTimeS;


        envelope =
            SmoothStep01(
                u);
    }


    // ------------------------------------------------------------
    // Continuous oscillation
    // ------------------------------------------------------------

    const sf::Scalar
        oscillationTime =
            timeS
            -
            startTimeS;


    const sf::Scalar
        phi =
            2.0
            *
            kPi
            *
            frequencyHz
            *
            oscillationTime;


    const sf::Scalar
        s =
            std::sin(
                phi);


    if (
        s >= 0.0)
    {
        // Right tendon pulls.
        tension[0] =
            0.0;


        tension[1] =
            envelope
            *
            amplitudeN
            *
            s;


        phase =
            1;
    }
    else
    {
        // Left tendon pulls.
        tension[0] =
            envelope
            *
            amplitudeN
            *
            (-s);


        tension[1] =
            0.0;


        phase =
            2;
    }


    return tension;
}


bool TendonTailActuator::IsFiniteVector(
    const sf::Vector3 &value)
{
    return std::isfinite(
               static_cast<double>(
                   value.x())) &&
           std::isfinite(
               static_cast<double>(
                   value.y())) &&
           std::isfinite(
               static_cast<double>(
                   value.z()));
}

void TendonTailActuator::ApplyPointForce(
    unsigned int linkIndex,
    const sf::Vector3 &worldPoint,
    const sf::Vector3 &worldForce)
{
    if (
        dynamics_ == nullptr ||
        linkIndex >=
            dynamics_->getNumOfLinks() ||
        !IsFiniteVector(
            worldPoint) ||
        !IsFiniteVector(
            worldForce))
    {
        safetyTripped_ =
            true;

        return;
    }

    // ============================================================
    // AddLinkForce acts at CG.
    //
    // A force F acting at physical point P is represented by:
    //
    //     Force at CG:
    //         F
    //
    //     Torque around CG:
    //         (P - CG) x F
    // ============================================================

    const sf::Vector3
        cgWorld =
            dynamics_->getLinkTransform(
                         linkIndex)
                .getOrigin();

    const sf::Vector3
        leverArm =
            worldPoint -
            cgWorld;

    const sf::Vector3
        torqueWorld =
            leverArm.cross(
                worldForce);

    dynamics_->AddLinkForce(
        linkIndex,
        worldForce);

    dynamics_->AddLinkTorque(
        linkIndex,
        torqueWorld);
}

void TendonTailActuator::ApplyTendonForces(
    const std::array<PathPoint, 6> &path,
    sf::Scalar tension)
{
    if (
        tension <= 0.0)
    {
        return;
    }

    std::array<sf::Vector3, 6>
        nodeForces;

    for (
        auto &force : nodeForces)
    {
        force =
            sf::Vector3(
                0.0,
                0.0,
                0.0);
    }

    // ============================================================
    // Every cable segment pulls its two endpoints toward each other.
    //
    // For:
    //
    //      Pi ------------ Pj
    //
    // unit vector:
    //
    //      u = (Pj - Pi) / |Pj - Pi|
    //
    // then:
    //
    //      F_i += +T u
    //      F_j += -T u
    //
    // For an intermediate guide the two adjacent segment forces
    // automatically add to the correct guide reaction.
    // ============================================================

    for (
        std::size_t i = 0;
        i + 1 < path.size();
        ++i)
    {
        const sf::Vector3
            segment =
                path[i + 1].world -
                path[i].world;

        const sf::Scalar
            segmentLength =
                segment.length();

        if (
            segmentLength <=
                kTinyLength ||
            !std::isfinite(
                static_cast<double>(
                    segmentLength)))
        {
            safetyTripped_ =
                true;

            return;
        }

        const sf::Vector3
            direction =
                segment /
                segmentLength;

        const sf::Vector3
            segmentForce =
                tension *
                direction;

        nodeForces[i] +=
            segmentForce;

        nodeForces[i + 1] -=
            segmentForce;
    }

    // ============================================================
    // IMPORTANT:
    //
    // path[0] is the fixed anchor on Body.
    //
    // Body is fixed in this diagnostic, therefore we deliberately
    // do NOT apply the anchor reaction.
    //
    // Only forces on Tail0..Tail4 are applied.
    //
    // This completely removes M1 / crank / shaft dynamics from
    // the experiment.
    // ============================================================

    for (
        std::size_t i = 0;
        i < path.size();
        ++i)
    {
        ApplyPointForce(
            path[i].linkIndex,
            path[i].world,
            nodeForces[i]);

        if (
            safetyTripped_)
        {
            return;
        }
    }
}

void TendonTailActuator::Update(
    sf::Scalar timeStep)
{
    sf::Actuator::Update(
        timeStep);

    if (
        dynamics_ == nullptr ||
        timeStep <= 0.0)
    {
        safetyTripped_ =
            true;

        snapshot_.safetyTripped =
            true;

        return;
    }

    elapsedTimeS_ +=
        timeStep;

    snapshot_.timeS =
        elapsedTimeS_;

    // ============================================================
    // 1. Read passive tail state
    // ============================================================

    std::array<sf::Scalar, 5>
        q{};

    std::array<sf::Scalar, 5>
        qDot{};

    if (
        !ReadJointState(
            q,
            qDot))
    {
        safetyTripped_ =
            true;
    }

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        snapshot_.jointPositionRad[i] =
            q[i];

        snapshot_.jointVelocityRadS[i] =
            qDot[i];

        if (
            std::abs(
                q[i]) >
            parameters_.jointSafetyLimitRad)
        {
            safetyTripped_ =
                true;
        }
    }

    // ============================================================
    // 2. Build current tendon geometry
    // ============================================================

    std::array<
        std::array<PathPoint, 6>,
        2>
        paths =
            {
                BuildTendonPath(
                    true),

                BuildTendonPath(
                    false)};

    for (
        std::size_t side = 0;
        side < 2;
        ++side)
    {
        const sf::Scalar
            length =
                ComputePathLength(
                    paths[side]);

        if (
            !std::isfinite(
                static_cast<double>(
                    length)) ||
            length <= 0.0)
        {
            safetyTripped_ =
                true;
        }

        snapshot_.tendonLengthM[side] =
            length;

        snapshot_.initialTendonLengthM[side] =
            initialTendonLengthM_[side];

        snapshot_.tendonLengthChangeM[side] =
            length -
            initialTendonLengthM_[side];
    }

    // ============================================================
    // 3. Prescribed direct tendon tension
    // ============================================================

    unsigned int
        phase =
            0;

    std::array<sf::Scalar, 2>
        commandedTension =
            ComputeDirectTestTensions(
                elapsedTimeS_,
                phase);

    snapshot_.testPhase =
        phase;

    // Safety is latching.
    //
    // Once it trips, direct tendon force remains zero for the
    // rest of this run.

    if (
        safetyTripped_)
    {
        commandedTension[0] =
            0.0;

        commandedTension[1] =
            0.0;
    }

    snapshot_.commandedTensionN =
        commandedTension;

    // ============================================================
    // 4. Apply DIRECT tendon forces
    // ============================================================

    if (
        !safetyTripped_)
    {
        ApplyTendonForces(
            paths[0],
            commandedTension[0]);

        if (
            !safetyTripped_)
        {
            ApplyTendonForces(
                paths[1],
                commandedTension[1]);
        }
    }

    // ============================================================
    // 5. Passive tail elasticity
    //
    // This is the ONLY direct joint torque.
    //
    // Tendon force enters through Tail link spatial forces.
    // ============================================================

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        const sf::Scalar
            passiveTorque =
                -parameters_.passiveStiffnessNmRad *
                    q[i]

                -

                parameters_.passiveDampingNmsRad *
                    qDot[i];

        dynamics_->DriveJoint(
            tailJointIndices_[i],
            passiveTorque);

        snapshot_.passiveTorqueNm[i] =
            passiveTorque;
    }

    snapshot_.safetyTripped =
        safetyTripped_;
}

TendonTailActuator::Snapshot
TendonTailActuator::GetSnapshot() const
{
    return snapshot_;
}
