#pragma once

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/entities/FeatherstoneEntity.h>

#include <array>
#include <string>


namespace sf
{
class Servo;
}


struct TendonTailParameters
{
    // ============================================================
    // Passive flexible tail
    // ============================================================

    sf::Scalar passiveStiffnessNmRad = 0.65;
    sf::Scalar passiveDampingNmsRad = 0.0;


    // ============================================================
    // Tendon model
    //
    // First R2-B implementation:
    //
    //     extension = L - L_free
    //
    //     if extension <= 0:
    //         tension = 0
    //
    //     if extension > 0:
    //         tension = max(
    //             0,
    //             k * extension
    //             +
    //             c * length_rate)
    //
    // The cable can pull but cannot push.
    // ============================================================

    sf::Scalar tendonStiffnessNPerM = 20000.0;
    sf::Scalar tendonDampingNsPerM = 10.0;

    // Initial cable pre-tension.
    //
    // Keep zero for the first diagnostic.
    // It can be changed later after the geometry is verified.
    sf::Scalar initialPretensionN = 0.0;


    // ============================================================
    // Diagnostic cable strain safety
    //
    // The project reference records approximately 3% extension.
    //
    // We do NOT interpret this as a measured breaking strain.
    // For R2-B it is only a "we have left the trusted region"
    // diagnostic threshold.
    // ============================================================

    sf::Scalar maxDiagnosticStrain = 0.03;


    // ============================================================
    // M1 diagnostic command
    // ============================================================

    sf::Scalar motorTargetFrequencyHz = 0.05;

    sf::Scalar motorStartTimeS = 1.0;
    sf::Scalar motorRampTimeS = 1.0;


    // XW540-T140-R reference at 12 V.
    //
    // This is a momentary maximum/stall reference, not a continuous
    // operating torque recommendation.
    sf::Scalar motorMaxTorqueNm = 6.9;

    // 72 rpm at 12 V ~= 7.5398 rad/s.
    sf::Scalar motorMaxVelocityRadS =
        7.5398223686155035;


    // ============================================================
    // Tail safety
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

        sf::Scalar motorCommandVelocityRadS = 0.0;
        sf::Scalar motorAngleRad = 0.0;
        sf::Scalar motorVelocityRadS = 0.0;
        sf::Scalar motorEffortTorqueNm = 0.0;

        bool motorSaturated = false;


        // [0] = left
        // [1] = right

        std::array<sf::Scalar, 2>
            tendonLengthM {};

        std::array<sf::Scalar, 2>
            tendonFreeLengthM {};

        std::array<sf::Scalar, 2>
            tendonExtensionM {};

        std::array<sf::Scalar, 2>
            tendonStrain {};

        std::array<sf::Scalar, 2>
            tendonLengthRateMS {};

        std::array<sf::Scalar, 2>
            tendonTensionN {};

        std::array<bool, 2>
            tendonOverstretch {
                false,
                false
            };


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
        sf::Servo* motorServo,
        unsigned int motorShaftLinkIndex,
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


    sf::Servo*
        motorServo_ =
            nullptr;


    unsigned int
        motorShaftLinkIndex_ =
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
        freeLengthM_ {};


    std::array<sf::Scalar, 2>
        previousLengthM_ {};


    bool
        previousLengthValid_ =
            false;


    bool
        safetyTripped_ =
            false;
};
