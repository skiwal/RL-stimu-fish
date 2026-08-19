#include "inc/tendon_tail_actuator.h"

#include <algorithm>
#include <cmath>


namespace
{

constexpr sf::Scalar kPi =
    3.1415926535897932384626433832795;


/*
    ================================================================
    Geometry used in Stage R2-A
    ================================================================

    IMPORTANT:

    These coordinates are NOT arbitrary visual guesses.

    Tail joint positions come from the CURRENT RL-stimu-fish SCN.

    Motor/crank/tendon guide parameters come from the source fishsim
    configuration already preserved in bionic_fish_v1_config.yaml.

    Coordinate convention here is the current robot BODY frame:

        +X = head / forward
        -X = tail
        +Z = down

    Tail joints rotate around +Z.

    Reference fishsim motor rotates around +Y.
*/


// ------------------------------------------------------------
// Current RL-stimu-fish TailJoint0 position in Body.
// ------------------------------------------------------------

constexpr sf::Scalar kJoint0X =
    -0.1810;


// ------------------------------------------------------------
// Current SCN distances:
// J0 -> J1
// J1 -> J2
// J2 -> J3
// J3 -> J4
//
// Tail direction is -X.
// ------------------------------------------------------------

constexpr std::array<sf::Scalar, 4>
    kJointSpacing =
    {
        0.0295,
        0.0310,
        0.0305,
        0.0250
    };


// ------------------------------------------------------------
// Tendon-guide longitudinal position from each joint.
//
// Derived from the original fishsim support geometry.
//
// Tail0:
//     body-tail0 / 2 + tailSegmentLength - 0.003
//
// etc.
//
// In our coordinates these extend along -X.
// ------------------------------------------------------------

constexpr std::array<sf::Scalar, 5>
    kGuideTailwardDistance =
    {
        0.0190,
        0.0195,
        0.0205,
        0.0190,
        0.0150
    };


// ------------------------------------------------------------
// Tendon lateral offsets from source fishsim.
// ------------------------------------------------------------

constexpr std::array<sf::Scalar, 5>
    kGuideLateralOffset =
    {
        0.045,
        0.035,
        0.025,
        0.015,
        0.005
    };


// ------------------------------------------------------------
// 0 = stay on nominal side
// 1 = cross to opposite side
// ------------------------------------------------------------

constexpr std::array<int, 5>
    kRouting =
    {
        0,
        0,
        0,
        1,
        1
    };


// ------------------------------------------------------------
// Motor location.
//
// fishsim:
//     MotorJoint X = 0.1655
//     Tail Joint0 X = 0.2850
//
// separation:
//     0.1195 m
//
// Current robot:
//     TailJoint0 X = -0.1810
//
// The motor is 0.1195 m toward the head (+X):
//
//     -0.1810 + 0.1195 = -0.0615
//
// This is therefore source-derived, not guessed.
// ------------------------------------------------------------

constexpr sf::Scalar kMotorPivotX =
    -0.0615;


// ------------------------------------------------------------
// Source-derived crank/tendon anchors.
//
// Left:
//     (0, -0.055, -0.0345)
//
// Right:
//     (0, +0.055, +0.0345)
//
// They rotate with the motor around +Y.
// ------------------------------------------------------------

constexpr TendonTailActuator::Vec3
    kLeftMotorAnchorLocal =
    {
        0.0,
        -0.055,
        -0.0345
    };


constexpr TendonTailActuator::Vec3
    kRightMotorAnchorLocal =
    {
        0.0,
        0.055,
        0.0345
    };

} // namespace


TendonTailActuator::TendonTailActuator(
    const std::string& name,
    sf::FeatherstoneEntity* dynamics,
    const std::array<unsigned int, 5>& jointIndices,
    const TendonTailParameters& parameters)
    : sf::Actuator(name),
      dynamics_(dynamics),
      jointIndices_(jointIndices),
      parameters_(parameters)
{
    /*
        MuJoCo spatial tendon default springlength = -1 means
        reference configuration length.

        Our reference configuration is:

            alpha = 0
            q0...q4 = 0

        So use exactly that configuration as the R2-A rest length.
    */

    const std::array<sf::Scalar, 5>
        qZero =
        {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        };


    restLengthM_[0] =
        ComputeTendonLength(
            0.0,
            qZero,
            true);


    restLengthM_[1] =
        ComputeTendonLength(
            0.0,
            qZero,
            false);


    snapshot_.tendonRestLengthM =
        restLengthM_;
}


sf::ActuatorType
TendonTailActuator::getType() const
{
    /*
        Stonefish has no tendon actuator enum.

        MOTOR is only the generic actuator category.
    */

    return
        sf::ActuatorType::MOTOR;
}


bool
TendonTailActuator::ReadJointState(
    std::array<sf::Scalar, 5>& q,
    std::array<sf::Scalar, 5>& qDot) const
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
            jointIndices_[i],
            q[i],
            positionType);


        dynamics_->getJointVelocity(
            jointIndices_[i],
            qDot[i],
            velocityType);


        if (
            positionType
                != btMultibodyLink::eRevolute
            ||
            velocityType
                != btMultibodyLink::eRevolute
            ||
            !std::isfinite(
                static_cast<double>(
                    q[i]))
            ||
            !std::isfinite(
                static_cast<double>(
                    qDot[i])))
        {
            return false;
        }
    }


    return true;
}


void
TendonTailActuator::Update(
    sf::Scalar timeStep)
{
    sf::Actuator::Update(
        timeStep);


    if (
        dynamics_ == nullptr
        ||
        timeStep <= 0.0)
    {
        return;
    }


    elapsedTimeS_ +=
        timeStep;


    std::array<sf::Scalar, 5>
        q {};


    std::array<sf::Scalar, 5>
        qDot {};


    if (
        !ReadJointState(
            q,
            qDot))
    {
        snapshot_.safetyTripped =
            true;

        motorVelocityRadS_ =
            0.0;

        return;
    }


    // ============================================================
    // Joint safety check
    // ============================================================

    for (
        sf::Scalar value
        : q)
    {
        if (
            std::abs(
                value)
            >
            parameters_.jointSafetyLimitRad)
        {
            snapshot_.safetyTripped =
                true;
        }
    }


    // ============================================================
    // Stage R2-A ideal motor shaft.
    //
    // This is deliberately a kinematic velocity source.
    //
    // R2-B will replace this with a real MotorRotor + MotorJoint.
    // ============================================================

    sf::Scalar
        commandedOmega =
        2.0
        * kPi
        * parameters_.motorTargetFrequencyHz;


    sf::Scalar
        ramp =
        0.0;


    if (
        elapsedTimeS_
        >=
        parameters_.motorStartTimeS)
    {
        if (
            parameters_.motorRampTimeS
            <=
            0.0)
        {
            ramp =
                1.0;
        }
        else
        {
            ramp =
                (
                    elapsedTimeS_
                    -
                    parameters_.motorStartTimeS
                )
                /
                parameters_.motorRampTimeS;


            ramp =
                std::max(
                    sf::Scalar(0.0),
                    std::min(
                        sf::Scalar(1.0),
                        ramp));
        }
    }


    if (
        snapshot_.safetyTripped)
    {
        motorVelocityRadS_ =
            0.0;
    }
    else
    {
        motorVelocityRadS_ =
            ramp
            * commandedOmega;
    }


    motorAngleRad_ +=
        motorVelocityRadS_
        * timeStep;


    // ============================================================
    // Tendon geometry and Jacobians
    // ============================================================

    std::array<sf::Scalar, 2>
        length {};


    std::array<sf::Scalar, 2>
        lengthRate {};


    std::array<sf::Scalar, 2>
        tendonForce {};


    std::array<
        std::array<sf::Scalar, 5>,
        2>
        jointJacobian {};


    std::array<sf::Scalar, 2>
        motorJacobian {};


    for (
        std::size_t side = 0;
        side < 2;
        ++side)
    {
        const bool
            left =
            side == 0;


        length[side] =
            ComputeTendonLength(
                motorAngleRad_,
                q,
                left);


        jointJacobian[side] =
            ComputeJointLengthJacobian(
                motorAngleRad_,
                q,
                left);


        motorJacobian[side] =
            ComputeMotorLengthDerivative(
                motorAngleRad_,
                q,
                left);


        lengthRate[side] =
            motorJacobian[side]
            * motorVelocityRadS_;


        for (
            std::size_t i = 0;
            i < 5;
            ++i)
        {
            lengthRate[side] +=
                jointJacobian[side][i]
                * qDot[i];
        }


        const sf::Scalar
            stretch =
            length[side]
            -
            restLengthM_[side];


        /*
            MuJoCo-compatible linear tendon spring/damper:

                F =
                    -k * (L-L0)
                    -c * Ldot

            This is a SIGNED scalar generalized tendon force.

            Do not silently reinterpret this as a pull-only cable.
            That hardware refinement comes later.
        */

        tendonForce[side] =
            -parameters_.tendonStiffnessNPerM
                * stretch

            -parameters_.tendonDampingNsPerM
                * lengthRate[side];
    }


    // ============================================================
    // Generalized motor reaction torque
    // ============================================================

    const sf::Scalar
        tendonGeneralizedMotorTorque =
            motorJacobian[0]
                * tendonForce[0]

            +
            motorJacobian[1]
                * tendonForce[1];


    const sf::Scalar
        motorRequiredTorque =
            -tendonGeneralizedMotorTorque;


    if (
        std::abs(
            motorRequiredTorque)
        >
        parameters_.motorReactionTorqueLimitNm)
    {
        /*
            Do not fake additional motor authority.

            Freeze the ideal motor shaft and flag the test.
        */

        snapshot_.safetyTripped =
            true;

        motorVelocityRadS_ =
            0.0;
    }


    // ============================================================
    // Apply passive spine + tendon generalized torques
    // ============================================================

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        const sf::Scalar
            passiveTorque =
                -parameters_.passiveStiffnessNmRad
                    * q[i]

                -parameters_.passiveDampingNmsRad
                    * qDot[i];


        const sf::Scalar
            tendonTorque =
                jointJacobian[0][i]
                    * tendonForce[0]

                +
                jointJacobian[1][i]
                    * tendonForce[1];


        const sf::Scalar
            totalTorque =
                passiveTorque
                +
                tendonTorque;


        /*
            This is an INTERNAL generalized joint torque.

            No body-level propulsion force is applied.
        */

        dynamics_->DriveJoint(
            jointIndices_[i],
            totalTorque);


        snapshot_.passiveTorqueNm[i] =
            passiveTorque;


        snapshot_.tendonTorqueNm[i] =
            tendonTorque;


        snapshot_.totalTorqueNm[i] =
            totalTorque;
    }


    // ============================================================
    // Telemetry snapshot
    // ============================================================

    snapshot_.timeS =
        elapsedTimeS_;


    snapshot_.motorAngleRad =
        motorAngleRad_;


    snapshot_.motorVelocityRadS =
        motorVelocityRadS_;


    snapshot_.motorRequiredTorqueNm =
        motorRequiredTorque;


    snapshot_.jointPositionRad =
        q;


    snapshot_.jointVelocityRadS =
        qDot;


    snapshot_.tendonLengthM =
        length;


    snapshot_.tendonLengthRateMS =
        lengthRate;


    snapshot_.tendonForceN =
        tendonForce;


    snapshot_.tendonRestLengthM =
        restLengthM_;


    for (
        std::size_t side = 0;
        side < 2;
        ++side)
    {
        snapshot_.tendonStretchM[side] =
            length[side]
            -
            restLengthM_[side];
    }
}


TendonTailActuator::Snapshot
TendonTailActuator::GetSnapshot() const
{
    return snapshot_;
}


// ============================================================================
// Tendon length
// ============================================================================

sf::Scalar
TendonTailActuator::ComputeTendonLength(
    sf::Scalar motorAngle,
    const std::array<sf::Scalar, 5>& q,
    bool leftTendon) const
{
    const Vec3
        motorAnchor =
            ComputeMotorAnchor(
                motorAngle,
                leftTendon);


    const std::array<Vec3, 5>
        guides =
            ComputeTendonGuidePoints(
                q,
                leftTendon);


    sf::Scalar
        length =
            Norm(
                Subtract(
                    guides[0],
                    motorAnchor));


    for (
        std::size_t i = 0;
        i < 4;
        ++i)
    {
        length +=
            Norm(
                Subtract(
                    guides[i + 1],
                    guides[i]));
    }


    return length;
}


// ============================================================================
// dL / dq
// ============================================================================

std::array<sf::Scalar, 5>
TendonTailActuator::ComputeJointLengthJacobian(
    sf::Scalar motorAngle,
    const std::array<sf::Scalar, 5>& q,
    bool leftTendon) const
{
    std::array<sf::Scalar, 5>
        jacobian {};


    const sf::Scalar
        epsilon =
            parameters_.jacobianEpsilonRad;


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        std::array<sf::Scalar, 5>
            qPlus =
                q;


        std::array<sf::Scalar, 5>
            qMinus =
                q;


        qPlus[i] +=
            epsilon;


        qMinus[i] -=
            epsilon;


        const sf::Scalar
            lPlus =
                ComputeTendonLength(
                    motorAngle,
                    qPlus,
                    leftTendon);


        const sf::Scalar
            lMinus =
                ComputeTendonLength(
                    motorAngle,
                    qMinus,
                    leftTendon);


        jacobian[i] =
            (
                lPlus
                -
                lMinus
            )
            /
            (
                2.0
                * epsilon
            );
    }


    return jacobian;
}


// ============================================================================
// dL / dalpha
// ============================================================================

sf::Scalar
TendonTailActuator::ComputeMotorLengthDerivative(
    sf::Scalar motorAngle,
    const std::array<sf::Scalar, 5>& q,
    bool leftTendon) const
{
    const sf::Scalar
        epsilon =
            parameters_.jacobianEpsilonRad;


    const sf::Scalar
        lPlus =
            ComputeTendonLength(
                motorAngle + epsilon,
                q,
                leftTendon);


    const sf::Scalar
        lMinus =
            ComputeTendonLength(
                motorAngle - epsilon,
                q,
                leftTendon);


    return
        (
            lPlus
            -
            lMinus
        )
        /
        (
            2.0
            * epsilon
        );
}


// ============================================================================
// Motor anchor
// ============================================================================

TendonTailActuator::Vec3
TendonTailActuator::ComputeMotorAnchor(
    sf::Scalar motorAngle,
    bool leftTendon) const
{
    const Vec3
        local =
            leftTendon
                ? kLeftMotorAnchorLocal
                : kRightMotorAnchorLocal;


    const Vec3
        rotated =
            RotateY(
                local,
                motorAngle);


    return
        Add(
            {
                kMotorPivotX,
                0.0,
                0.0
            },
            rotated);
}


// ============================================================================
// Tail tendon guide points
// ============================================================================

std::array<TendonTailActuator::Vec3, 5>
TendonTailActuator::ComputeTendonGuidePoints(
    const std::array<sf::Scalar, 5>& q,
    bool leftTendon) const
{
    std::array<Vec3, 5>
        jointPosition {};


    std::array<sf::Scalar, 5>
        absoluteAngle {};


    jointPosition[0] =
        {
            kJoint0X,
            0.0,
            0.0
        };


    sf::Scalar
        theta =
            0.0;


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        theta +=
            q[i];


        absoluteAngle[i] =
            theta;


        if (
            i < 4)
        {
            jointPosition[i + 1] =
                Add(
                    jointPosition[i],
                    RotateZ(
                        {
                            -kJointSpacing[i],
                            0.0,
                            0.0
                        },
                        theta));
        }
    }


    std::array<Vec3, 5>
        guide {};


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        /*
            fishsim convention:

                left  = negative Y
                right = positive Y
        */

        sf::Scalar
            side =
                leftTendon
                    ? -1.0
                    : 1.0;


        if (
            kRouting[i] != 0)
        {
            side =
                -side;
        }


        const Vec3
            localGuide =
            {
                -kGuideTailwardDistance[i],

                side
                    * kGuideLateralOffset[i],

                0.0
            };


        guide[i] =
            Add(
                jointPosition[i],
                RotateZ(
                    localGuide,
                    absoluteAngle[i]));
    }


    return guide;
}


// ============================================================================
// Vector helpers
// ============================================================================

TendonTailActuator::Vec3
TendonTailActuator::RotateZ(
    const Vec3& v,
    sf::Scalar angle)
{
    const sf::Scalar
        c =
            std::cos(
                angle);


    const sf::Scalar
        s =
            std::sin(
                angle);


    return
    {
        c * v.x
            - s * v.y,

        s * v.x
            + c * v.y,

        v.z
    };
}


TendonTailActuator::Vec3
TendonTailActuator::RotateY(
    const Vec3& v,
    sf::Scalar angle)
{
    const sf::Scalar
        c =
            std::cos(
                angle);


    const sf::Scalar
        s =
            std::sin(
                angle);


    return
    {
        c * v.x
            + s * v.z,

        v.y,

        -s * v.x
            + c * v.z
    };
}


TendonTailActuator::Vec3
TendonTailActuator::Add(
    const Vec3& a,
    const Vec3& b)
{
    return
    {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}


TendonTailActuator::Vec3
TendonTailActuator::Subtract(
    const Vec3& a,
    const Vec3& b)
{
    return
    {
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    };
}


sf::Scalar
TendonTailActuator::Norm(
    const Vec3& v)
{
    return
        std::sqrt(
            v.x * v.x
            +
            v.y * v.y
            +
            v.z * v.z);
}
