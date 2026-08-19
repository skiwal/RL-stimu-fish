#pragma once

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/entities/FeatherstoneEntity.h>

#include <array>
#include <string>


struct TendonTailParameters
{
    // ------------------------------------------------------------
    // Passive flexible spine
    // ------------------------------------------------------------

    sf::Scalar passiveStiffnessNmRad = 0.65;
    sf::Scalar passiveDampingNmsRad = 0.0;


    // ------------------------------------------------------------
    // fishsim tendon parameters
    //
    // Source model:
    //     stiffness = 20000
    //     damping   = 10
    //
    // For the spatial tendon these are treated as:
    //     N/m
    //     N*s/m
    // ------------------------------------------------------------

    sf::Scalar tendonStiffnessNPerM = 20000.0;
    sf::Scalar tendonDampingNsPerM = 10.0;


    // ------------------------------------------------------------
    // Stage R2-A motor command
    //
    // Very slow first diagnostic:
    //
    //     0.05 Hz = 1 revolution / 20 s
    //
    // The reference robot is driven substantially faster, but this
    // first test is deliberately quasi-static.
    // ------------------------------------------------------------

    sf::Scalar motorTargetFrequencyHz = 0.05;

    sf::Scalar motorStartTimeS = 1.0;
    sf::Scalar motorRampTimeS = 1.0;


    // ------------------------------------------------------------
    // Hardware safety reference.
    //
    // XW540-T140-R 12 V stall torque:
    //     6.9 Nm
    //
    // R2-A uses an ideal velocity source, but if the tendon geometry
    // would require more than this reaction torque, the diagnostic
    // trips and stops advancing the motor.
    // ------------------------------------------------------------

    sf::Scalar motorReactionTorqueLimitNm = 6.9;


    // ------------------------------------------------------------
    // Joint safety
    //
    // The XML mechanical limit is +/-80 deg.
    // Stop the R2-A motor earlier if the model approaches extreme
    // deformation.
    // ------------------------------------------------------------

    sf::Scalar jointSafetyLimitRad =
        1.0471975511965976; // 60 deg


    // ------------------------------------------------------------
    // Numerical derivative for tendon Jacobian.
    // ------------------------------------------------------------

    sf::Scalar jacobianEpsilonRad = 1.0e-6;
};


class TendonTailActuator final
    : public sf::Actuator
{
public:

    struct Snapshot
    {
        sf::Scalar timeS = 0.0;

        sf::Scalar motorAngleRad = 0.0;
        sf::Scalar motorVelocityRadS = 0.0;

        /*
            Positive value means the ideal motor has to provide
            positive torque to oppose the tendon reaction.
        */
        sf::Scalar motorRequiredTorqueNm = 0.0;


        // [0] left
        // [1] right
        std::array<sf::Scalar, 2> tendonLengthM {};
        std::array<sf::Scalar, 2> tendonRestLengthM {};
        std::array<sf::Scalar, 2> tendonStretchM {};
        std::array<sf::Scalar, 2> tendonLengthRateMS {};

        /*
            Signed scalar tendon spring/damper force.

            This intentionally reproduces the mathematical behaviour
            of the fishsim/MuJoCo tendon spring for R2-A.

            It is NOT yet a pull-only/slack fishing-line model.
        */
        std::array<sf::Scalar, 2> tendonForceN {};


        std::array<sf::Scalar, 5> jointPositionRad {};
        std::array<sf::Scalar, 5> jointVelocityRadS {};

        std::array<sf::Scalar, 5> passiveTorqueNm {};
        std::array<sf::Scalar, 5> tendonTorqueNm {};
        std::array<sf::Scalar, 5> totalTorqueNm {};


        bool safetyTripped = false;
    };


    TendonTailActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,
        const std::array<unsigned int, 5>& jointIndices,
        const TendonTailParameters& parameters);


    sf::ActuatorType
    getType() const override;


    void
    Update(
        sf::Scalar timeStep) override;


    Snapshot
    GetSnapshot() const;


public:

    struct Vec3
    {
        sf::Scalar x = 0.0;
        sf::Scalar y = 0.0;
        sf::Scalar z = 0.0;
    };

private:

    bool
    ReadJointState(
        std::array<sf::Scalar, 5>& q,
        std::array<sf::Scalar, 5>& qDot) const;


    sf::Scalar
    ComputeTendonLength(
        sf::Scalar motorAngle,
        const std::array<sf::Scalar, 5>& q,
        bool leftTendon) const;


    std::array<sf::Scalar, 5>
    ComputeJointLengthJacobian(
        sf::Scalar motorAngle,
        const std::array<sf::Scalar, 5>& q,
        bool leftTendon) const;


    sf::Scalar
    ComputeMotorLengthDerivative(
        sf::Scalar motorAngle,
        const std::array<sf::Scalar, 5>& q,
        bool leftTendon) const;


    Vec3
    ComputeMotorAnchor(
        sf::Scalar motorAngle,
        bool leftTendon) const;


    std::array<Vec3, 5>
    ComputeTendonGuidePoints(
        const std::array<sf::Scalar, 5>& q,
        bool leftTendon) const;


    static Vec3
    RotateZ(
        const Vec3& v,
        sf::Scalar angle);


    static Vec3
    RotateY(
        const Vec3& v,
        sf::Scalar angle);


    static Vec3
    Add(
        const Vec3& a,
        const Vec3& b);


    static Vec3
    Subtract(
        const Vec3& a,
        const Vec3& b);


    static sf::Scalar
    Norm(
        const Vec3& v);


private:

    sf::FeatherstoneEntity* dynamics_ =
        nullptr;


    std::array<unsigned int, 5>
        jointIndices_ {};


    TendonTailParameters
        parameters_;


    Snapshot
        snapshot_;


    sf::Scalar
        elapsedTimeS_ =
        0.0;


    sf::Scalar
        motorAngleRad_ =
        0.0;


    sf::Scalar
        motorVelocityRadS_ =
        0.0;


    std::array<sf::Scalar, 2>
        restLengthM_ {};
};
