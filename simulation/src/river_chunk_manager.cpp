#include "inc/river_chunk_manager.h"

#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/entities/animation/ManualTrajectory.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <utility>

RiverChunkManager::RiverChunkManager(
    RiverChunkConfig config)
    : config_(std::move(config))
{
    if (config_.chunkCount < 3)
    {
        throw std::invalid_argument(
            "chunkCount must be at least 3.");
    }

    if (config_.chunkCount % 2 == 0)
    {
        throw std::invalid_argument(
            "chunkCount must be an odd number.");
    }

    if (config_.chunkLength <= 0.0
        || config_.riverWidth <= 0.0
        || config_.riverDepth <= 0.0)
    {
        throw std::invalid_argument(
            "River dimensions must be positive.");
    }

    const int requiredChunks =
        config_.keepChunksAhead
        + config_.keepChunksBehind
        + 1;

    if (config_.chunkCount < requiredChunks)
    {
        throw std::invalid_argument(
            "chunkCount is too small for the requested margins.");
    }
}

void RiverChunkManager::Build(
    sf::SimulationManager& simulation)
{
    simulation_ = &simulation;

    chunks_.clear();
    chunks_.reserve(
        static_cast<std::size_t>(config_.chunkCount));

    const int halfCount = config_.chunkCount / 2;

    // 初始逻辑区块编号：
    // 7 个区块时为 -3, -2, -1, 0, 1, 2, 3
    const std::int64_t firstGlobalIndex =
        -static_cast<std::int64_t>(halfCount);

    // 轻微重叠，避免区块边界出现可见缝隙
    constexpr sf::Scalar overlap = 0.04;

    const sf::Vector3 bedDimensions(
        config_.chunkLength + overlap,
        config_.riverWidth,
        config_.bedThickness);

    const sf::Vector3 bankDimensions(
        config_.chunkLength + overlap,
        config_.bankThickness,
        config_.bankHeight);

    for (int slot = 0;
         slot < config_.chunkCount;
         ++slot)
    {
        RiverChunk chunk;

        chunk.globalIndex =
            firstGlobalIndex
            + static_cast<std::int64_t>(slot);

        chunk.centerX =
            static_cast<sf::Scalar>(
                slot - halfCount)
            * config_.chunkLength;

        const std::string suffix =
            std::to_string(slot);

        chunk.bed = CreateBox(
            "RiverBed_" + suffix,
            bedDimensions,
            "RiverBedLook",
            sf::I4());

        chunk.positiveYBank = CreateBox(
            "RiverBankPositiveY_" + suffix,
            bankDimensions,
            "RiverBankLook",
            sf::I4());

        chunk.negativeYBank = CreateBox(
            "RiverBankNegativeY_" + suffix,
            bankDimensions,
            "RiverBankLook",
            sf::I4());

        SetChunkPose(chunk);

        chunks_.push_back(chunk);
    }

    nextRearGlobalIndex_ =
        firstGlobalIndex - 1;

    nextFrontGlobalIndex_ =
        firstGlobalIndex
        + static_cast<std::int64_t>(
            config_.chunkCount);

    std::cout
        << "[RiverChunkManager] Created "
        << chunks_.size()
        << " reusable chunks.\n";

    std::cout
        << "[RiverChunkManager] Active river length: "
        << config_.chunkLength
               * static_cast<sf::Scalar>(
                   config_.chunkCount)
        << " m\n";
}

void RiverChunkManager::Update(
    sf::Scalar fishX)
{
    if (chunks_.empty())
    {
        return;
    }

    const sf::Scalar requiredAheadDistance =
        static_cast<sf::Scalar>(
            config_.keepChunksAhead)
        * config_.chunkLength;

    const sf::Scalar requiredBehindDistance =
        static_cast<sf::Scalar>(
            config_.keepChunksBehind)
        * config_.chunkLength;

    /*
        鱼向 +X 上游运动：

        当鱼到最前方区块的距离不足时，
        将最末端区块移动到最前方。
    */
    while (
        GetFrontMostX() - fishX
        < requiredAheadDistance)
    {
        RecycleRearChunkToFront();
    }

    /*
        鱼向 -X 下游运动：

        当鱼到最后方区块的距离不足时，
        将最前方区块移动到最后方。
    */
    while (
        fishX - GetRearMostX()
        < requiredBehindDistance)
    {
        RecycleFrontChunkToRear();
    }
}

RiverChunkManager::MovingBox
RiverChunkManager::CreateBox(
    const std::string& name,
    const sf::Vector3& dimensions,
    const std::string& look,
    const sf::Transform& initialTransform)
{
    if (simulation_ == nullptr)
    {
        throw std::logic_error(
            "RiverChunkManager has no SimulationManager.");
    }

    auto* trajectory =
        new sf::ManualTrajectory();

    trajectory->setTransform(initialTransform);

    // 区块平时完全静止
    trajectory->setLinearVelocity(sf::V0());
    trajectory->setAngularVelocity(sf::V0());

    auto* entity =
        new sf::AnimatedEntity(
            name,
            trajectory,
            dimensions,
            sf::I4(),
            "RiverRock",
            look,
            true);

    simulation_->AddAnimatedEntity(entity);

    return MovingBox{
        entity,
        trajectory
    };
}

void RiverChunkManager::SetChunkPose(
    RiverChunk& chunk)
{
    /*
        河床表面位于 Z = riverDepth。

        例如：
        riverDepth = 4.0
        bedThickness = 0.5

        河床中心 Z = 4.25
    */
    const sf::Scalar bedCenterZ =
        config_.riverDepth
        + config_.bedThickness * 0.5;

    /*
        河岸从水面上方 0.5 m 延伸到河床下方 0.5 m。

        depth = 4
        bankHeight = 5
        bankCenterZ = 2
    */
    const sf::Scalar bankCenterZ =
        (config_.riverDepth - 0.5) * 0.5;

    /*
        河流内部宽度为 riverWidth。

        河岸内表面位于：
        Y = ±riverWidth / 2
    */
    const sf::Scalar bankCenterY =
        config_.riverWidth * 0.5
        + config_.bankThickness * 0.5;

    SetBoxPose(
        chunk.bed,
        chunk.centerX,
        0.0,
        bedCenterZ);

    SetBoxPose(
        chunk.positiveYBank,
        chunk.centerX,
        bankCenterY,
        bankCenterZ);

    SetBoxPose(
        chunk.negativeYBank,
        chunk.centerX,
        -bankCenterY,
        bankCenterZ);
}

void RiverChunkManager::SetBoxPose(
    MovingBox& box,
    sf::Scalar x,
    sf::Scalar y,
    sf::Scalar z)
{
    if (box.trajectory == nullptr)
    {
        throw std::runtime_error(
            "River chunk trajectory is null.");
    }

    box.trajectory->setTransform(
        sf::Transform(
            sf::IQ(),
            sf::Vector3(x, y, z)));

    // 搬运完成后区块仍然保持静止
    box.trajectory->setLinearVelocity(sf::V0());
    box.trajectory->setAngularVelocity(sf::V0());
}

void RiverChunkManager::RecycleRearChunkToFront()
{
    RiverChunk& rearChunk =
        GetRearMostChunk();

    const sf::Scalar oldX =
        rearChunk.centerX;

    rearChunk.centerX =
        GetFrontMostX()
        + config_.chunkLength;

    rearChunk.globalIndex =
        nextFrontGlobalIndex_;

    ++nextFrontGlobalIndex_;

    SetChunkPose(rearChunk);

    std::cout
        << "[RiverChunkManager] Rear -> front, "
        << "global index = "
        << rearChunk.globalIndex
        << ", X: "
        << oldX
        << " -> "
        << rearChunk.centerX
        << '\n';
}

void RiverChunkManager::RecycleFrontChunkToRear()
{
    RiverChunk& frontChunk =
        GetFrontMostChunk();

    const sf::Scalar oldX =
        frontChunk.centerX;

    frontChunk.centerX =
        GetRearMostX()
        - config_.chunkLength;

    frontChunk.globalIndex =
        nextRearGlobalIndex_;

    --nextRearGlobalIndex_;

    SetChunkPose(frontChunk);

    std::cout
        << "[RiverChunkManager] Front -> rear, "
        << "global index = "
        << frontChunk.globalIndex
        << ", X: "
        << oldX
        << " -> "
        << frontChunk.centerX
        << '\n';
}

RiverChunkManager::RiverChunk&
RiverChunkManager::GetRearMostChunk()
{
    if (chunks_.empty())
    {
        throw std::runtime_error(
            "No river chunks exist.");
    }

    return *std::min_element(
        chunks_.begin(),
        chunks_.end(),
        [](const RiverChunk& left,
           const RiverChunk& right)
        {
            return left.centerX < right.centerX;
        });
}

RiverChunkManager::RiverChunk&
RiverChunkManager::GetFrontMostChunk()
{
    if (chunks_.empty())
    {
        throw std::runtime_error(
            "No river chunks exist.");
    }

    return *std::max_element(
        chunks_.begin(),
        chunks_.end(),
        [](const RiverChunk& left,
           const RiverChunk& right)
        {
            return left.centerX < right.centerX;
        });
}

sf::Scalar RiverChunkManager::GetRearMostX() const
{
    if (chunks_.empty())
    {
        throw std::runtime_error(
            "No river chunks exist.");
    }

    const auto iterator =
        std::min_element(
            chunks_.begin(),
            chunks_.end(),
            [](const RiverChunk& left,
               const RiverChunk& right)
            {
                return left.centerX < right.centerX;
            });

    return iterator->centerX;
}

sf::Scalar RiverChunkManager::GetFrontMostX() const
{
    if (chunks_.empty())
    {
        throw std::runtime_error(
            "No river chunks exist.");
    }

    const auto iterator =
        std::max_element(
            chunks_.begin(),
            chunks_.end(),
            [](const RiverChunk& left,
               const RiverChunk& right)
            {
                return left.centerX < right.centerX;
            });

    return iterator->centerX;
}
