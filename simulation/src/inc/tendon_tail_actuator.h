#pragma once

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/entities/FeatherstoneEntity.h>

#include <array>
#include <string>


struct TendonTailParameters
{
    // ============================================================
    // Passive flexible tail
    // ============================================================

    sf::Scalar passiveStiffnessNmRad = 0.65;

    sf::Scalar passiveDampingNmsRad = 0.0;


    // ============================================================
    // Direct tendon force diagnostic
    //
    // No motor.
    // No crank dynamics.
    // No tendon elasticity -> tension conversion.
    //
    // We directly prescribe tendon tension.
    // ============================================================

    sf::Scalar directTestTensionN = 1.0;


    // Timeline:
    //
    // 0 ~ 1 s     neutral
    //
    // 1 ~ 2 s     right tendon ramp 0 -> 1 N
    // 2 ~ 3 s     right tendon hold 1 N
    // 3 ~ 4 s     right tendon ramp 1 -> 0 N
    //
    // 4 ~ 5 s     neutral
    //
    // 5 ~ 6 s     left tendon ramp 0 -> 1 N
    // 6 ~ 7 s     left tendon hold 1 N
    // 7 ~ 8 s     left tendon ramp 1 -> 0 N
    //
    // > 8 s       neutral

    sf::Scalar initialSettleTimeS = 1.0;

    sf::Scalar rampTimeS = 1.0;

    sf::Scalar holdTimeS = 1.0;

    sf::Scalar centerPauseTimeS = 1.0;


    // ============================================================
    // Tail diagnostic safety
    // ============================================================

    sf::Scalar jointSafetyLimitRad =
        1.0471975511965976; // 60 deg
};


class TendonTailActuator final
    : public sf::Actuator
{
public:

    struct Snapshot
    {
        sf::Scalar timeS = 0.0;


        // --------------------------------------------------------
        // Test phase
        //
        // 0 = initial neutral
        // 1 = right ramp up
        // 2 = right hold
        // 3 = right ramp down
        // 4 = center pause
        // 5 = left ramp up
        // 6 = left hold
        // 7 = left ramp down
        // 8 = finished / neutral
        // --------------------------------------------------------

        unsigned int testPhase = 0;


        // [0] = left
        // [1] = right

        std::array<sf::Scalar, 2>
            commandedTensionN {};


        std::array<sf::Scalar, 2>
            tendonLengthM {};


        std::array<sf::Scalar, 2>
            initialTendonLengthM {};


        std::array<sf::Scalar, 2>
            tendonLengthChangeM {};


        std::array<sf::Scalar, 5>
            jointPositionRad {};


        std::array<sf::Scalar, 5>
            jointVelocityRadS {};


        std::array<sf::Scalar, 5>
            passiveTorqueNm {};


        bool safetyTripped = false;
    };


    TendonTailActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,
        unsigned int bodyLinkIndex,
        const std::array<unsigned int, 5>& tailLinkIndices,
        const std::array<unsigned int, 5>& tailJointIndices,
        const TendonTailParameters& parameters);


    sf::ActuatorType
    getType() const override;


    void
    Update(
        sf::Scalar timeStep) override;


    Snapshot
    GetSnapshot() const;


private:

    struct PathPoint
    {
        unsigned int linkIndex = 0;

        sf::Vector3 world =
            sf::Vector3(
                0.0,
                0.0,
                0.0);
    };


    bool
    ReadJointState(
        std::array<sf::Scalar, 5>& q,
        std::array<sf::Scalar, 5>& qDot) const;


    sf::Vector3
    LocalPointToWorld(
        unsigned int linkIndex,
        const sf::Vector3& localPoint) const;


    std::array<PathPoint, 6>
    BuildTendonPath(
        bool leftTendon) const;


    static sf::Scalar
    ComputePathLength(
        const std::array<PathPoint, 6>& path);


    static sf::Scalar
    SmoothStep01(
        sf::Scalar x);


    std::array<sf::Scalar, 2>
    ComputeDirectTestTensions(
        sf::Scalar timeS,
        unsigned int& phase) const;


    void
    ApplyTendonForces(
        const std::array<PathPoint, 6>& path,
        sf::Scalar tension);


    void
    ApplyPointForce(
        unsigned int linkIndex,
        const sf::Vector3& worldPoint,
        const sf::Vector3& worldForce);


    static bool
    IsFiniteVector(
        const sf::Vector3& value);


private:

    sf::FeatherstoneEntity*
        dynamics_ =
            nullptr;


    unsigned int
        bodyLinkIndex_ =
            0;


    std::array<unsigned int, 5>
        tailLinkIndices_ {};


    std::array<unsigned int, 5>
        tailJointIndices_ {};


    TendonTailParameters
        parameters_;


    Snapshot
        snapshot_;


    sf::Scalar
        elapsedTimeS_ =
            0.0;


    std::array<sf::Scalar, 2>
        initialTendonLengthM_ {};


    bool
        safetyTripped_ =
            false;
};
