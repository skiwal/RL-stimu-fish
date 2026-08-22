#pragma once

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/entities/FeatherstoneEntity.h>

#include <array>
#include <string>

struct TendonTailParameters
{
    // Original fishsim passive hinge parameters.
    sf::Scalar passiveStiffnessNmRad = 0.65;
    sf::Scalar passiveDampingNmsRad = 0.0;

    // S-bend diagnostic:
    // LEFT tendon = constant 1 N
    // RIGHT tendon = 0 N
    sf::Scalar diagnosticLeftTensionN = 1.0;

    // Same ±80 deg tail safety envelope as current model.
    sf::Scalar jointSafetyLimitRad = 1.3962634016;

    // Numerical derivative for -T dL/dq validation.
    sf::Scalar jacobianEpsilonRad = 1.0e-5;
};

class TendonTailActuator final : public sf::Actuator
{
public:
    struct Snapshot
    {
        sf::Scalar timeS = 0.0;

        std::array<sf::Scalar,2> commandedTensionN{};
        std::array<sf::Scalar,2> tendonLengthM{};
        std::array<sf::Scalar,2> initialTendonLengthM{};

        std::array<sf::Scalar,5> jointPositionRad{};
        std::array<sf::Scalar,5> jointVelocityRadS{};
        std::array<sf::Scalar,5> passiveTorqueNm{};

        // Force applied by LEFT tendon at Tail0..Tail4 guides.
        std::array<sf::Vector3,5> guideForceWorld{};

        // Generalized tendon torque from actual point forces.
        std::array<sf::Scalar,5> tendonTorqueFromForcesNm{};

        // Independent check: -T dL/dq.
        std::array<sf::Scalar,5> tendonTorqueFromJacobianNm{};

        std::array<sf::Scalar,5> tendonTorqueErrorNm{};

        sf::Vector3 bodyAnchorForceWorld{0,0,0};

        // Whole virtual tendon conservation check.
        sf::Vector3 tendonNetForceWorld{0,0,0};
        sf::Vector3 tendonNetTorqueWorld{0,0,0};

        sf::Scalar jacobianMaxErrorNm = 0.0;

        bool safetyTripped = false;
    };

    TendonTailActuator(
        const std::string& name,
        sf::FeatherstoneEntity* dynamics,
        unsigned int bodyLinkIndex,
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

    using Path = std::array<PathPoint,6>;

    bool ReadJointState(
        std::array<sf::Scalar,5>& q,
        std::array<sf::Scalar,5>& qDot) const;

    sf::Vector3 LocalPointToWorld(
        unsigned int link,
        const sf::Vector3& local) const;

    Path BuildTendonPath(bool left) const;

    static sf::Scalar ComputePathLength(const Path& path);
    static bool IsFiniteVector(const sf::Vector3& v);

    bool JointAxisPivot(
        std::size_t joint,
        sf::Vector3& axisWorld,
        sf::Vector3& pivotWorld) const;

    sf::Scalar PerturbedPathLength(
        const Path& path,
        std::size_t joint,
        sf::Scalar delta) const;

    void ValidateTendon(
        const Path& path,
        const std::array<sf::Vector3,6>& nodeForces,
        sf::Scalar tension);

    void ApplyPointForce(
        unsigned int link,
        const sf::Vector3& point,
        const sf::Vector3& force);

    void ApplyTendonForces(
        const Path& path,
        sf::Scalar tension);

    sf::FeatherstoneEntity* dynamics_ = nullptr;

    unsigned int bodyLinkIndex_ = 0;

    std::array<unsigned int,5> tailLinkIndices_{};
    std::array<unsigned int,5> tailJointIndices_{};

    TendonTailParameters parameters_;
    Snapshot snapshot_;

    sf::Scalar elapsedTimeS_ = 0.0;

    std::array<sf::Scalar,2> initialTendonLengthM_{};

    bool safetyTripped_ = false;
};
