#include "inc/tendon_tail_actuator.h"

#include <Stonefish/actuators/Servo.h>
#include <Stonefish/entities/SolidEntity.h>

#include <algorithm>
#include <cmath>

namespace
{

    constexpr sf::Scalar kPi =
        3.1415926535897932384626433832795;

    constexpr sf::Scalar kTinyLength =
        1.0e-9;

    // ================================================================
    // Tendon guide geometry
    //
    // These points are expressed in each Tail link BODY-ORIGIN frame.
    //
    // Each Tail link origin is located at the corresponding revolute
    // joint, therefore:
    //
    //     local X = negative = toward tail
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

    // 0 = nominal side
    // 1 = crossed to opposite side

    constexpr std::array<int, 5>
        kRouting =
            {
                0,
                0,
                0,
                1,
                1};

    // ================================================================
    // Virtual crank / tendon anchors.
    //
    // The MotorShaft rotates physically in Stonefish around +Y.
    //
    // The XZ radial vectors of these two attachment points are opposite,
    // therefore they are 180 degrees apart around the shaft.
    //
    // Note:
    //
    //     physical crank arm length reference = 39.5 mm
    //
    // The actual tendon attachment site in the reference geometry is
    // inset by 5 mm:
    //
    //     39.5 - 5.0 = 34.5 mm
    //
    // ================================================================

    const sf::Vector3
        kLeftMotorAnchorLocal(
            0.0,
            -0.055,
            -0.0345);

    const sf::Vector3
        kRightMotorAnchorLocal(
            0.0,
            0.055,
            0.0345);

} // namespace

TendonTailActuator::TendonTailActuator(
    const std::string &name,
    sf::FeatherstoneEntity *dynamics,
    sf::Servo *motorServo,
    unsigned int motorShaftLinkIndex,
    const std::array<unsigned int, 5> &tailLinkIndices,
    const std::array<unsigned int, 5> &tailJointIndices,
    const TendonTailParameters &parameters)
    : sf::Actuator(
          name),
      dynamics_(
          dynamics),
      motorServo_(
          motorServo),
      motorShaftLinkIndex_(
          motorShaftLinkIndex),
      tailLinkIndices_(
          tailLinkIndices),
      tailJointIndices_(
          tailJointIndices),
      parameters_(
          parameters)
{
    if (
        dynamics_ == nullptr ||
        motorServo_ == nullptr)
    {
        safetyTripped_ =
            true;

        snapshot_.safetyTripped =
            true;

        return;
    }

    // ------------------------------------------------------------
    // Determine cable free length from the actual neutral geometry.
    //
    // For optional pretension:
    //
    //     T0 = k * (L_initial - L_free)
    //
    // therefore:
    //
    //     L_free = L_initial - T0/k
    //
    // ------------------------------------------------------------

    for (
        std::size_t side = 0;
        side < 2;
        ++side)
    {
        const bool
            left =
                side == 0;

        const auto
            path =
                BuildTendonPath(
                    left);

        const sf::Scalar
            initialLength =
                ComputePathLength(
                    path);

        sf::Scalar
            freeLength =
                initialLength;

        if (
            parameters_.tendonStiffnessNPerM >
            0.0)
        {
            freeLength -=
                parameters_.initialPretensionN /
                parameters_.tendonStiffnessNPerM;
        }

        freeLength =
            std::max(
                freeLength,
                kTinyLength);

        freeLengthM_[side] =
            freeLength;

        previousLengthM_[side] =
            initialLength;

        snapshot_.tendonLengthM[side] =
            initialLength;

        snapshot_.tendonFreeLengthM[side] =
            freeLength;

        snapshot_.tendonExtensionM[side] =
            initialLength -
            freeLength;

        snapshot_.tendonStrain[side] =
            snapshot_.tendonExtensionM[side] /
            freeLength;
    }

    previousLengthValid_ =
        true;
}

sf::ActuatorType
TendonTailActuator::getType() const
{
    // Stonefish has no native tendon actuator type.
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

    // IMPORTANT:
    //
    // Guide coordinates are defined in BODY ORIGIN coordinates,
    // not in CG coordinates.
    //
    // Therefore use getOTransform(), not getLinkTransform().

    return link.solid->getOTransform() *
           localPoint;
}

std::array<TendonTailActuator::PathPoint, 6>
TendonTailActuator::BuildTendonPath(
    bool leftTendon) const
{
    std::array<PathPoint, 6>
        path;

    // ------------------------------------------------------------
    // P0 = crank anchor on real MotorShaft
    // ------------------------------------------------------------

    path[0].linkIndex =
        motorShaftLinkIndex_;

    path[0].world =
        LocalPointToWorld(
            motorShaftLinkIndex_,
            leftTendon
                ? kLeftMotorAnchorLocal
                : kRightMotorAnchorLocal);

    // ------------------------------------------------------------
    // P1..P5 = Tail0..Tail4 guides / final anchor
    // ------------------------------------------------------------

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
        total =
            0.0;

    for (
        std::size_t i = 0;
        i + 1 < path.size();
        ++i)
    {
        const sf::Vector3
            delta =
                path[i + 1].world -
                path[i].world;

        total +=
            delta.length();
    }

    return total;
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

    // ------------------------------------------------------------
    // Stonefish AddLinkForce() applies force at link CG.
    //
    // A force F physically acting at point P is equivalent to:
    //
    //     Force at CG:
    //         F
    //
    //     Torque at CG:
    //         (P - CG) x F
    //
    // ------------------------------------------------------------

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

    // ------------------------------------------------------------
    // Each cable segment pulls both of its endpoints toward the
    // opposite endpoint.
    //
    // For segment:
    //
    //     Pi -------- Pj
    //
    // force on Pi:
    //
    //     +T * unit(Pj-Pi)
    //
    // force on Pj:
    //
    //     -T * unit(Pj-Pi)
    //
    // Intermediate guide forces therefore emerge automatically by
    // summing the two adjacent segment forces.
    // ------------------------------------------------------------

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
            kTinyLength)
        {
            safetyTripped_ =
                true;

            continue;
        }

        const sf::Vector3
            direction =
                segment /
                segmentLength;

        const sf::Vector3
            segmentForce =
                direction *
                tension;

        nodeForces[i] +=
            segmentForce;

        nodeForces[i + 1] -=
            segmentForce;
    }

    for (
        std::size_t i = 0;
        i < path.size();
        ++i)
    {
        ApplyPointForce(
            path[i].linkIndex,
            path[i].world,
            nodeForces[i]);
    }
}

void TendonTailActuator::Update(
    sf::Scalar timeStep)
{
    sf::Actuator::Update(
        timeStep);

    if (
        dynamics_ == nullptr ||
        motorServo_ == nullptr ||
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
    // 1. Read tail joint state
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
    // 2. M1 velocity command
    // ============================================================

    const sf::Scalar
        targetOmega =
            2.0 *
            kPi *
            parameters_.motorTargetFrequencyHz;

    sf::Scalar
        ramp =
            0.0;

    if (
        elapsedTimeS_ >=
        parameters_.motorStartTimeS)
    {
        if (
            parameters_.motorRampTimeS <=
            0.0)
        {
            ramp =
                1.0;
        }
        else
        {
            ramp =
                (elapsedTimeS_ -
                 parameters_.motorStartTimeS) /
                parameters_.motorRampTimeS;

            ramp =
                std::max(
                    sf::Scalar(0.0),
                    std::min(
                        sf::Scalar(1.0),
                        ramp));
        }
    }

    sf::Scalar
        motorCommand =
            ramp *
            targetOmega;

    if (
        safetyTripped_)
    {
        motorServo_->setDesiredVelocity(0.0);
        motorServo_->setMaxTorque(0.0);

        snapshot_.motorCommandVelocityRadS = 0.0;
        snapshot_.tendonTensionN[0] = 0.0;
        snapshot_.tendonTensionN[1] = 0.0;
        snapshot_.safetyTripped = true;

        return;
    }

    motorServo_->setDesiredVelocity(
        motorCommand);

    snapshot_.motorCommandVelocityRadS =
        motorCommand;

    snapshot_.motorAngleRad =
        motorServo_->getPosition();

    snapshot_.motorVelocityRadS =
        motorServo_->getVelocity();

    snapshot_.motorEffortTorqueNm =
        motorServo_->getEffort();

    snapshot_.motorSaturated =
        std::abs(
            snapshot_.motorEffortTorqueNm) >=
        0.98 *
            parameters_.motorMaxTorqueNm;

    // ============================================================
    // 3. Actual spatial tendon geometry
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

    std::array<sf::Scalar, 2>
        currentLength{};

    std::array<sf::Scalar, 2>
        lengthRate{};

    std::array<sf::Scalar, 2>
        tension{};

    for (
        std::size_t side = 0;
        side < 2;
        ++side)
    {
        currentLength[side] =
            ComputePathLength(
                paths[side]);

        if (
            !std::isfinite(
                static_cast<double>(
                    currentLength[side])) ||
            currentLength[side] <=
                0.0)
        {
            safetyTripped_ =
                true;

            currentLength[side] =
                previousLengthM_[side];
        }

        if (
            previousLengthValid_)
        {
            lengthRate[side] =
                (currentLength[side] -
                 previousLengthM_[side]) /
                timeStep;
        }
        else
        {
            lengthRate[side] =
                0.0;
        }

        const sf::Scalar
            extension =
                currentLength[side] -
                freeLengthM_[side];

        sf::Scalar
            strain =
                0.0;

        if (
            freeLengthM_[side] >
            kTinyLength)
        {
            strain =
                extension /
                freeLengthM_[side];
        }

        const bool
            overstretch =
                parameters_.maxDiagnosticStrain >
                    0.0 &&
                strain >
                    parameters_.maxDiagnosticStrain;

        snapshot_.tendonLengthM[side] =
            currentLength[side];

        snapshot_.tendonFreeLengthM[side] =
            freeLengthM_[side];

        snapshot_.tendonExtensionM[side] =
            extension;

        snapshot_.tendonStrain[side] =
            strain;

        snapshot_.tendonLengthRateMS[side] =
            lengthRate[side];

        snapshot_.tendonOverstretch[side] =
            overstretch;

        if (
            overstretch)
        {
            safetyTripped_ =
                true;
        }

        // ========================================================
        // Pull-only cable.
        //
        // A cable shorter than its free length is slack.
        //
        // It NEVER produces compression/pushing force.
        // ========================================================

        if (
            extension >
            0.0)
        {
            sf::Scalar
                extensionForForce =
                    extension;

            // ----------------------------------------------------
            // Numerical safety:
            //
            // If geometry has already crossed the diagnostic
            // strain threshold, DO NOT alter the actual geometry.
            //
            // We only cap the extension used in the emergency
            // force calculation, while simultaneously stopping M1.
            //
            // This prevents an unbounded k*x impulse from exploding
            // the simulation.
            // ----------------------------------------------------

            if (
                parameters_.maxDiagnosticStrain >
                0.0)
            {
                const sf::Scalar
                    maxExtensionForForce =
                        parameters_.maxDiagnosticStrain *
                        freeLengthM_[side];

                extensionForForce =
                    std::min(
                        extensionForForce,
                        maxExtensionForForce);
            }

            const sf::Scalar
                rawTension =
                    parameters_.tendonStiffnessNPerM *
                        extensionForForce

                    +

                    parameters_.tendonDampingNsPerM *
                        lengthRate[side];

            tension[side] =
                std::max(
                    sf::Scalar(0.0),
                    rawTension);
        }
        else
        {
            tension[side] =
                0.0;
        }

        if (
            !std::isfinite(
                static_cast<double>(
                    tension[side])))
        {
            tension[side] =
                0.0;

            safetyTripped_ =
                true;
        }

        snapshot_.tendonTensionN[side] =
            tension[side];
    }

    previousLengthM_ =
        currentLength;

    previousLengthValid_ =
        true;

    // ============================================================
    // 4. If a safety condition appeared during tendon calculation,
    //    stop M1 immediately.
    // ============================================================

    if (
        safetyTripped_)
    {
        motorServo_->setDesiredVelocity(0.0);
        motorServo_->setMaxTorque(0.0);

        snapshot_.motorCommandVelocityRadS = 0.0;
        snapshot_.tendonTensionN[0] = 0.0;
        snapshot_.tendonTensionN[1] = 0.0;
        snapshot_.safetyTripped = true;

        return;
    }

    // ============================================================
    // 5. Apply actual spatial tendon forces.
    //
    // LEFT and RIGHT cables each act on:
    //
    //     MotorShaft crank anchor
    //     Tail0 guide
    //     Tail1 guide
    //     Tail2 guide
    //     Tail3 guide
    //     Tail4 final anchor
    //
    // No manual tendon joint torque is generated here.
    // ============================================================

    ApplyTendonForces(
        paths[0],
        tension[0]);

    ApplyTendonForces(
        paths[1],
        tension[1]);

    // ============================================================
    // 6. Passive flexible spine
    //
    // The ONLY directly applied J0~J4 torque is passive elasticity.
    //
    // Tendon loading reaches the joints through the rigid-body
    // dynamics solved by Stonefish.
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
