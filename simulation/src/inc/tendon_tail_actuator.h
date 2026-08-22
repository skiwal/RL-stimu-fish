#pragma once

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/entities/FeatherstoneEntity.h>

#include <array>
#include <string>

struct TendonTailParameters
{
    // fishsim passive tail
    sf::Scalar passiveStiffnessNmRad = 0.65;
    sf::Scalar passiveDampingNmsRad = 0.0;

    // fishsim spatial tendon
    sf::Scalar tendonStiffnessNPerM = 20000.0;
    sf::Scalar tendonDampingNsPerM = 10.0;

    // Diagnostic protection only.
    // Not a physical force clamp.
    sf::Scalar tendonForceSafetyLimitN = 2000.0;

    // Original joint range ±80 deg.
    sf::Scalar jointSafetyLimitRad = 1.3962634016;
};


class TendonTailActuator final :
    public sf::Actuator
{
public:

    struct Snapshot
    {
        sf::Scalar timeS = 0.0;

        sf::Scalar motorPositionRad = 0.0;
        sf::Scalar motorVelocityRadS = 0.0;

        // [0] left, [1] right
        std::array<sf::Scalar,2> tendonLengthM{};
        std::array<sf::Scalar,2> restLengthM{};
        std::array<sf::Scalar,2> tendonDeltaLengthM{};
        std::array<sf::Scalar,2> tendonLengthVelocityMS{};

        // Signed MuJoCo-style spring/damper force.
        //
        // + = tensile / pulling
        // - = compressed linear spring / pushing
        std::array<sf::Scalar,2> tendonForceN{};

        std::array<sf::Scalar,5> jointPositionRad{};
        std::array<sf::Scalar,5> jointVelocityRadS{};
        std::array<sf::Scalar,5> passiveTorqueNm{};

        // [side][joint]
        std::array<
            std::array<sf::Scalar,5>,
            2
        > tendonTorqueNm{};

        std::array<sf::Scalar,5>
            totalTendonTorqueNm{};

        // [side][Tail0..Tail4]
        std::array<
            std::array<sf::Vector3,5>,
            2
        > guideForceWorld{};

        std::array<sf::Vector3,2>
            motorAnchorForceWorld{
                sf::Vector3(0,0,0),
                sf::Vector3(0,0,0)
            };

        sf::Vector3 tendonNetForceResidualWorld{0,0,0};
        sf::Vector3 tendonNetTorqueResidualWorld{0,0,0};

        bool safetyTripped = false;
    };


    TendonTailActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,

        unsigned int motorLinkIndex,
        unsigned int motorJointIndex,

        const std::array<unsigned int,5>& tailLinkIndices,
        const std::array<unsigned int,5>& tailJointIndices,

        const TendonTailParameters& parameters);


    sf::ActuatorType getType() const override;

    void Update(sf::Scalar timeStep) override;

    Snapshot GetSnapshot() const;


private:

    struct PathPoint
    {
        unsigned int linkIndex = 0;
        sf::Vector3 world{0,0,0};
    };

    using Path =
        std::array<PathPoint,6>;


    bool ReadRevoluteJoint(
        unsigned int jointIndex,
        sf::Scalar& q,
        sf::Scalar& qDot) const;


    sf::Vector3 LocalPointToWorld(
        unsigned int linkIndex,
        const sf::Vector3& localPoint) const;


    Path BuildTendonPath(
        bool left) const;


    static sf::Scalar ComputePathLength(
        const Path& path);


    static bool IsFiniteVector(
        const sf::Vector3& v);


    bool JointAxisPivot(
        std::size_t joint,
        sf::Vector3& axisWorld,
        sf::Vector3& pivotWorld) const;


    void ApplyPointForce(
        unsigned int linkIndex,
        const sf::Vector3& worldPoint,
        const sf::Vector3& worldForce);


    void ApplySignedTendon(
        const Path& path,
        sf::Scalar signedForceN,
        std::size_t side);


    sf::FeatherstoneEntity* dynamics_ = nullptr;

    unsigned int motorLinkIndex_ = 0;
    unsigned int motorJointIndex_ = 0;

    std::array<unsigned int,5>
        tailLinkIndices_{};

    std::array<unsigned int,5>
        tailJointIndices_{};

    TendonTailParameters parameters_;

    Snapshot snapshot_;

    sf::Scalar elapsedTimeS_ = 0.0;

    std::array<sf::Scalar,2>
        restLengthM_{};

    std::array<sf::Scalar,2>
        previousLengthM_{};

    bool lengthHistoryReady_ = false;
    bool safetyTripped_ = false;
};
