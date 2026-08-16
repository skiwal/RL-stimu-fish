#include "inc/river_chunk_manager.h"

#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/entities/animation/ManualTrajectory.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <utility>


namespace
{

constexpr int kPlacementAttempts = 80;


/*
    uniform_real_distribution 要求 min <= max。

    如果两者相同，直接返回 min。
*/
sf::Scalar RandomScalar(
    std::mt19937_64& random,
    sf::Scalar minimum,
    sf::Scalar maximum)
{
    if (maximum <= minimum)
    {
        return minimum;
    }

    std::uniform_real_distribution<sf::Scalar>
        distribution(
            minimum,
            maximum);

    return distribution(random);
}


int RandomInt(
    std::mt19937_64& random,
    int minimum,
    int maximum)
{
    if (maximum <= minimum)
    {
        return minimum;
    }

    std::uniform_int_distribution<int>
        distribution(
            minimum,
            maximum);

    return distribution(random);
}


sf::Scalar RandomSignedSpeed(
    std::mt19937_64& random,
    sf::Scalar minimum,
    sf::Scalar maximum)
{
    const sf::Scalar speed =
        RandomScalar(
            random,
            minimum,
            maximum);

    std::bernoulli_distribution
        directionDistribution(0.5);

    return directionDistribution(random)
        ? speed
        : -speed;
}


/*
    对往返运动进行反射。

    与简单 clamp 不同，即使未来 timeStep 较大，
    也不会让障碍物卡在边界。
*/
void ReflectAtBounds(
    sf::Scalar& position,
    sf::Scalar minimum,
    sf::Scalar maximum,
    sf::Scalar& speed)
{
    if (maximum <= minimum)
    {
        position = minimum;
        speed = 0.0;
        return;
    }

    while (
        position < minimum
        || position > maximum)
    {
        if (position > maximum)
        {
            position =
                maximum
                - (position - maximum);

            speed =
                -std::abs(speed);
        }

        if (position < minimum)
        {
            position =
                minimum
                + (minimum - position);

            speed =
                std::abs(speed);
        }
    }
}

} // namespace


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

    if (
        config_.chunkLength <= 0.0
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

    if (
        config_.minFixedObstaclesPerChunk < 0
        || config_.maxFixedObstaclesPerChunk
            < config_.minFixedObstaclesPerChunk)
    {
        throw std::invalid_argument(
            "Invalid fixed obstacle count.");
    }

    if (
        config_.minMovingObstaclesPerChunk < 0
        || config_.maxMovingObstaclesPerChunk
            < config_.minMovingObstaclesPerChunk)
    {
        throw std::invalid_argument(
            "Invalid moving obstacle count.");
    }

    if (
        config_.minRockRadius <= 0.0
        || config_.maxRockRadius
            < config_.minRockRadius)
    {
        throw std::invalid_argument(
            "Invalid rock radius range.");
    }

    if (
        config_.minRockExposedHeight <= 0.0
        || config_.maxRockHeight
            < config_.minRockExposedHeight)
    {
        throw std::invalid_argument(
            "Invalid rock height range.");
    }

    if (
        config_.safeCorridorWidth <= 0.0
        || config_.safeCorridorWidth
            >= config_.riverWidth)
    {
        throw std::invalid_argument(
            "safeCorridorWidth must be smaller than riverWidth.");
    }

    if (
        config_.movingObstacleMinSpeed <= 0.0
        || config_.movingObstacleMaxSpeed
            < config_.movingObstacleMinSpeed)
    {
        throw std::invalid_argument(
            "Invalid moving obstacle speed range.");
    }
}


void RiverChunkManager::Build(
    sf::SimulationManager& simulation)
{
    simulation_ =
        &simulation;

    chunks_.clear();

    chunks_.reserve(
        static_cast<std::size_t>(
            config_.chunkCount));


    const int halfCount =
        config_.chunkCount / 2;

    /*
        7 个 chunk：

        -3 -2 -1 0 1 2 3
    */
    const std::int64_t firstGlobalIndex =
        -static_cast<std::int64_t>(
            halfCount);


    /*
        防止两个 chunk 接缝出现视觉缝隙。
    */
    constexpr sf::Scalar overlap =
        0.04;


    const sf::Vector3 bedDimensions(
        config_.chunkLength + overlap,
        config_.riverWidth,
        config_.bedThickness);


    const sf::Vector3 bankDimensions(
        config_.chunkLength + overlap,
        config_.bankThickness,
        config_.bankHeight);


    for (
        int slot = 0;
        slot < config_.chunkCount;
        ++slot)
    {
        RiverChunk chunk;


        chunk.globalIndex =
            firstGlobalIndex
            + static_cast<std::int64_t>(
                slot);


        chunk.centerX =
            static_cast<sf::Scalar>(
                slot - halfCount)
            * config_.chunkLength;


        const std::string suffix =
            std::to_string(slot);


        // ========================================================
        // 河床
        // ========================================================

        chunk.bed =
            CreateBox(
                "RiverBed_" + suffix,

                bedDimensions,

                "RiverRock",

                "RiverBedLook",

                sf::I4());


        // ========================================================
        // 河岸
        // ========================================================

        chunk.positiveYBank =
            CreateBox(
                "RiverBankPositiveY_"
                    + suffix,

                bankDimensions,

                "RiverRock",

                "RiverBankLook",

                sf::I4());


        chunk.negativeYBank =
            CreateBox(
                "RiverBankNegativeY_"
                    + suffix,

                bankDimensions,

                "RiverRock",

                "RiverBankLook",

                sf::I4());


        // ========================================================
        // 障碍物对象池
        // ========================================================

        /*
            对象池的几何尺寸在 Build 时确定。

            chunk 回收时：

                不 new
                不 delete

            只修改位置和行为。
        */
        std::mt19937_64 poolRandom(
            config_.obstacleSeed
            ^ (
                static_cast<std::uint64_t>(
                    slot + 1)
                * 0x9E3779B97F4A7C15ULL));


        // --------------------------------------------------------
        // 固定岩石
        // --------------------------------------------------------

        chunk.fixedObstacles.reserve(
            static_cast<std::size_t>(
                config_
                    .maxFixedObstaclesPerChunk));


        for (
            int obstacleIndex = 0;
            obstacleIndex
                < config_
                    .maxFixedObstaclesPerChunk;
            ++obstacleIndex)
        {
            FixedObstacle obstacle;


            obstacle.radius =
                RandomScalar(
                    poolRandom,
                    config_.minRockRadius,
                    config_.maxRockRadius);


            obstacle.body =
                CreateCylinder(
                    "RiverRock_"
                        + suffix
                        + "_"
                        + std::to_string(
                            obstacleIndex),

                    obstacle.radius,

                    config_.maxRockHeight,

                    "RiverRock",

                    "ObstacleRockLook",

                    sf::I4());


            chunk.fixedObstacles.push_back(
                obstacle);
        }


        // --------------------------------------------------------
        // 移动“小鱼”
        // --------------------------------------------------------

        chunk.movingObstacles.reserve(
            static_cast<std::size_t>(
                config_
                    .maxMovingObstaclesPerChunk));


        for (
            int obstacleIndex = 0;
            obstacleIndex
                < config_
                    .maxMovingObstaclesPerChunk;
            ++obstacleIndex)
        {
            MovingObstacle obstacle;


            const sf::Scalar length =
                RandomScalar(
                    poolRandom,

                    config_
                        .movingFishMinLength,

                    config_
                        .movingFishMaxLength);


            const sf::Scalar width =
                RandomScalar(
                    poolRandom,

                    config_
                        .movingFishMinWidth,

                    config_
                        .movingFishMaxWidth);


            const sf::Scalar height =
                width
                * config_
                    .movingFishHeightRatio;


            obstacle.halfLength =
                length * 0.5;

            obstacle.halfWidth =
                width * 0.5;

            obstacle.halfHeight =
                height * 0.5;


            obstacle.body =
                CreateBox(
                    "MovingFish_"
                        + suffix
                        + "_"
                        + std::to_string(
                            obstacleIndex),

                    sf::Vector3(
                        length,
                        width,
                        height),

                    "FishMaterial",

                    "ObstacleFishLook",

                    sf::I4());


            chunk.movingObstacles.push_back(
                obstacle);
        }


        // ========================================================
        // 初始位置
        // ========================================================

        SetChunkPose(chunk);

        ConfigureChunkObstacles(
            chunk);


        chunks_.push_back(
            std::move(chunk));
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


    std::cout
        << "[RiverChunkManager] Fixed obstacle pool: "
        << config_.chunkCount
               * config_
                   .maxFixedObstaclesPerChunk
        << "\n";


    std::cout
        << "[RiverChunkManager] Moving obstacle pool: "
        << config_.chunkCount
               * config_
                   .maxMovingObstaclesPerChunk
        << "\n";
}


void RiverChunkManager::Update(
    sf::Scalar fishX,
    sf::Scalar timeStep)
{
    if (chunks_.empty())
    {
        return;
    }


    const sf::Scalar
        requiredAheadDistance =
            static_cast<sf::Scalar>(
                config_.keepChunksAhead)
            * config_.chunkLength;


    const sf::Scalar
        requiredBehindDistance =
            static_cast<sf::Scalar>(
                config_.keepChunksBehind)
            * config_.chunkLength;


    /*
        鱼向 +X 游。

        后方 chunk 搬到前方。
    */
    while (
        GetFrontMostX() - fishX
        < requiredAheadDistance)
    {
        RecycleRearChunkToFront();
    }


    /*
        鱼向 -X 游。

        前方 chunk 搬到后方。
    */
    while (
        fishX - GetRearMostX()
        < requiredBehindDistance)
    {
        RecycleFrontChunkToRear();
    }


    /*
        移动障碍物使用 Stonefish 的物理 timeStep，
        不依赖渲染帧率。
    */
    if (timeStep > 0.0)
    {
        for (RiverChunk& chunk : chunks_)
        {
            UpdateMovingObstacles(
                chunk,
                timeStep);
        }
    }
}


// ================================================================
// Entity creation
// ================================================================

RiverChunkManager::MovingBody
RiverChunkManager::CreateBox(
    const std::string& name,
    const sf::Vector3& dimensions,
    const std::string& material,
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


    trajectory->setTransform(
        initialTransform);

    trajectory->setLinearVelocity(
        sf::V0());

    trajectory->setAngularVelocity(
        sf::V0());


    /*
        collides = true

        AnimatedEntity 是 kinematic body，
        可以与主鱼这种 dynamic body 发生碰撞。
    */
    auto* entity =
        new sf::AnimatedEntity(
            name,

            trajectory,

            dimensions,

            sf::I4(),

            material,

            look,

            true);


    simulation_->AddAnimatedEntity(
        entity);


    return MovingBody{
        entity,
        trajectory
    };
}


RiverChunkManager::MovingBody
RiverChunkManager::CreateCylinder(
    const std::string& name,
    sf::Scalar radius,
    sf::Scalar height,
    const std::string& material,
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


    trajectory->setTransform(
        initialTransform);

    trajectory->setLinearVelocity(
        sf::V0());

    trajectory->setAngularVelocity(
        sf::V0());


    /*
        Stonefish cylinder 的轴线沿 Z。
        非常适合从河床向上伸出的石头。
    */
    auto* entity =
        new sf::AnimatedEntity(
            name,

            trajectory,

            radius,

            height,

            sf::I4(),

            material,

            look,

            true);


    simulation_->AddAnimatedEntity(
        entity);


    return MovingBody{
        entity,
        trajectory
    };
}


// ================================================================
// River geometry
// ================================================================

void RiverChunkManager::SetChunkPose(
    RiverChunk& chunk)
{
    /*
        NED：

            Z = 0             水面

            Z = riverDepth    河床表面
    */

    const sf::Scalar bedCenterZ =
        config_.riverDepth
        + config_.bedThickness * 0.5;


    /*
        河岸顶部位于：

            Z = -0.5

        默认：
            bankHeight = 5
            center = -0.5 + 2.5 = 2.0

        因此河岸范围：

            -0.5 ～ 4.5
    */
    const sf::Scalar bankTopZ =
        -0.5;


    const sf::Scalar bankCenterZ =
        bankTopZ
        + config_.bankHeight * 0.5;


    /*
        河道内部宽度 = riverWidth。

        河岸内表面位于：

            Y = ± riverWidth/2
    */
    const sf::Scalar bankCenterY =
        config_.riverWidth * 0.5
        + config_.bankThickness * 0.5;


    SetMovingBodyPose(
        chunk.bed,

        sf::Transform(
            sf::IQ(),

            sf::Vector3(
                chunk.centerX,
                0.0,
                bedCenterZ)),

        sf::V0(),

        sf::V0());


    SetMovingBodyPose(
        chunk.positiveYBank,

        sf::Transform(
            sf::IQ(),

            sf::Vector3(
                chunk.centerX,
                bankCenterY,
                bankCenterZ)),

        sf::V0(),

        sf::V0());


    SetMovingBodyPose(
        chunk.negativeYBank,

        sf::Transform(
            sf::IQ(),

            sf::Vector3(
                chunk.centerX,
                -bankCenterY,
                bankCenterZ)),

        sf::V0(),

        sf::V0());
}


void RiverChunkManager::SetMovingBodyPose(
    MovingBody& body,
    const sf::Transform& transform,
    const sf::Vector3& linearVelocity,
    const sf::Vector3& angularVelocity)
{
    if (body.trajectory == nullptr)
    {
        throw std::runtime_error(
            "Animated trajectory is null.");
    }


    body.trajectory->setTransform(
        transform);


    body.trajectory->setLinearVelocity(
        linearVelocity);


    body.trajectory->setAngularVelocity(
        angularVelocity);
}


// ================================================================
// Random generation
// ================================================================

std::uint64_t
RiverChunkManager::MakeChunkSeed(
    std::int64_t globalIndex) const
{
    /*
        SplitMix64 风格混合。

        让：

            -1
             0
             1
             2

        等相邻逻辑 chunk 得到完全不同的随机序列。
    */
    std::uint64_t value =
        static_cast<std::uint64_t>(
            globalIndex);


    value +=
        config_.obstacleSeed
        + 0x9E3779B97F4A7C15ULL;


    value =
        (value ^ (value >> 30))
        * 0xBF58476D1CE4E5B9ULL;


    value =
        (value ^ (value >> 27))
        * 0x94D049BB133111EBULL;


    value ^=
        value >> 31;


    return value;
}


void RiverChunkManager::ConfigureChunkObstacles(
    RiverChunk& chunk)
{
    std::mt19937_64 random(
        MakeChunkSeed(
            chunk.globalIndex));


    const sf::Scalar halfRiverWidth =
        config_.riverWidth * 0.5;


    const sf::Scalar halfChunkLength =
        config_.chunkLength * 0.5;


    // ============================================================
    // 安全通道
    // ============================================================

    const sf::Scalar
        safeCorridorHalfWidth =
            config_.safeCorridorWidth
            * 0.5;


    const sf::Scalar
        corridorCenterLimit =
            std::max(
                sf::Scalar(0.0),

                halfRiverWidth
                - config_
                    .obstacleBankClearance
                - safeCorridorHalfWidth);


    chunk.safeCorridorCenterY =
        RandomScalar(
            random,

            -corridorCenterLimit,

            corridorCenterLimit);


    // ============================================================
    // 固定石头
    // ============================================================

    const int fixedCount =
        RandomInt(
            random,

            config_
                .minFixedObstaclesPerChunk,

            config_
                .maxFixedObstaclesPerChunk);


    for (
        std::size_t index = 0;
        index
            < chunk.fixedObstacles.size();
        ++index)
    {
        FixedObstacle& obstacle =
            chunk.fixedObstacles[index];


        obstacle.active =
            false;


        if (
            static_cast<int>(index)
            >= fixedCount)
        {
            HideFixedObstacle(
                chunk,
                obstacle);

            continue;
        }


        const sf::Scalar radius =
            obstacle.radius;


        const sf::Scalar xMinimum =
            -halfChunkLength
            + config_
                .obstacleChunkEdgeClearance
            + radius;


        const sf::Scalar xMaximum =
            halfChunkLength
            - config_
                .obstacleChunkEdgeClearance
            - radius;


        const sf::Scalar yMinimum =
            -halfRiverWidth
            + config_
                .obstacleBankClearance
            + radius;


        const sf::Scalar yMaximum =
            halfRiverWidth
            - config_
                .obstacleBankClearance
            - radius;


        bool placed =
            false;


        for (
            int attempt = 0;
            attempt < kPlacementAttempts;
            ++attempt)
        {
            const sf::Scalar candidateX =
                RandomScalar(
                    random,
                    xMinimum,
                    xMaximum);


            const sf::Scalar candidateY =
                RandomScalar(
                    random,
                    yMinimum,
                    yMaximum);


            // ----------------------------------------------------
            // 保留整条安全通道
            // ----------------------------------------------------

            constexpr sf::Scalar
                corridorExtraClearance =
                    0.15;


            // if (
            //     std::abs(
            //         candidateY
            //         - chunk
            //             .safeCorridorCenterY)

            //     <
            //     safeCorridorHalfWidth
            //     + radius
            //     + corridorExtraClearance)
            // {
            //     continue;
            // }


            // ----------------------------------------------------
            // 初始主鱼周围留空
            // ----------------------------------------------------

            const sf::Scalar worldX =
                chunk.centerX
                + candidateX;


            const sf::Scalar clearRadius =
                config_
                    .initialObstacleClearRadius;


            if (
                worldX * worldX
                + candidateY * candidateY

                <
                clearRadius
                * clearRadius)
            {
                continue;
            }


            // ----------------------------------------------------
            // 固定石头之间不要重叠
            // ----------------------------------------------------

            bool overlapsRock =
                false;


            for (
                std::size_t previous = 0;
                previous < index;
                ++previous)
            {
                const FixedObstacle&
                    previousObstacle =
                        chunk
                            .fixedObstacles[
                                previous];


                if (
                    !previousObstacle.active)
                {
                    continue;
                }


                const sf::Scalar dx =
                    candidateX
                    - previousObstacle
                        .localX;


                const sf::Scalar dy =
                    candidateY
                    - previousObstacle
                        .localY;


                const sf::Scalar
                    minimumDistance =
                        radius
                        + previousObstacle
                            .radius
                        + 0.25;


                if (
                    dx * dx
                    + dy * dy

                    <
                    minimumDistance
                    * minimumDistance)
                {
                    overlapsRock =
                        true;

                    break;
                }
            }


            if (overlapsRock)
            {
                continue;
            }


            obstacle.localX =
                candidateX;

            obstacle.localY =
                candidateY;


            // ----------------------------------------------------
            // 高度
            // ----------------------------------------------------

            const bool
                canPierceSurface =
                    config_.maxRockHeight
                    >
                    config_.riverDepth
                    + 0.20;


            std::bernoulli_distribution
                piercingDistribution(
                    config_
                        .surfacePiercingRockProbability);


            const bool
                surfacePiercing =
                    canPierceSurface
                    && piercingDistribution(
                        random);


            if (surfacePiercing)
            {
                /*
                    确保顶部至少超过水面约 0.15 m。
                */
                const sf::Scalar
                    minimumPiercingHeight =
                        config_.riverDepth
                        + 0.15;


                obstacle.exposedHeight =
                    RandomScalar(
                        random,

                        minimumPiercingHeight,

                        config_
                            .maxRockHeight);
            }
            else
            {
                const sf::Scalar
                    submergedMaximum =
                        std::min(
                            config_
                                .maxRockHeight,

                            std::min(
                                config_
                                    .maxSubmergedRockExposedHeight,

                                config_
                                    .riverDepth
                                - 0.15));


                obstacle.exposedHeight =
                    RandomScalar(
                        random,

                        config_
                            .minRockExposedHeight,

                        std::max(
                            config_
                                .minRockExposedHeight,

                            submergedMaximum));
            }


            /*
                cylinder 总高度不改变。

                例如：

                    riverDepth = 4
                    maxRockHeight = 5.2
                    exposedHeight = 1.0

                石头顶部：

                    Z = 3

                中心：

                    3 + 2.6 = 5.6

                大部分 cylinder 被埋进河床。
            */
            const sf::Scalar rockCenterZ =
                config_.riverDepth
                + config_.maxRockHeight * 0.5
                - obstacle.exposedHeight;


            SetMovingBodyPose(
                obstacle.body,

                sf::Transform(
                    sf::IQ(),

                    sf::Vector3(
                        chunk.centerX
                            + obstacle.localX,

                        obstacle.localY,

                        rockCenterZ)),

                sf::V0(),

                sf::V0());


            obstacle.active =
                true;


            placed =
                true;


            break;
        }


        if (!placed)
        {
            HideFixedObstacle(
                chunk,
                obstacle);
        }
    }


    // ============================================================
    // 移动障碍物
    // ============================================================

    const int movingCount =
        RandomInt(
            random,

            config_
                .minMovingObstaclesPerChunk,

            config_
                .maxMovingObstaclesPerChunk);


    for (
        std::size_t index = 0;
        index
            < chunk.movingObstacles.size();
        ++index)
    {
        MovingObstacle& obstacle =
            chunk.movingObstacles[index];


        obstacle.active =
            false;


        if (
            static_cast<int>(index)
            >= movingCount)
        {
            HideMovingObstacle(
                chunk,
                obstacle);

            continue;
        }


        const sf::Scalar xMinimum =
            -halfChunkLength
            + config_
                .obstacleChunkEdgeClearance
            + obstacle.halfLength;


        const sf::Scalar xMaximum =
            halfChunkLength
            - config_
                .obstacleChunkEdgeClearance
            - obstacle.halfLength;


        const sf::Scalar yMinimum =
            -halfRiverWidth
            + config_
                .obstacleBankClearance
            + obstacle.halfWidth;


        const sf::Scalar yMaximum =
            halfRiverWidth
            - config_
                .obstacleBankClearance
            - obstacle.halfWidth;


        const sf::Scalar zMinimum =
            config_
                .movingObstacleVerticalClearance
            + obstacle.halfHeight;


        const sf::Scalar zMaximum =
            config_.riverDepth
            - config_
                .movingObstacleVerticalClearance
            - obstacle.halfHeight;


        bool placed =
            false;


        for (
            int attempt = 0;
            attempt < kPlacementAttempts;
            ++attempt)
        {
            const int motionType =
                RandomInt(
                    random,
                    0,
                    2);


            obstacle.motion =
                static_cast<ObstacleMotion>(
                    motionType);


            obstacle.speed =
                RandomSignedSpeed(
                    random,

                    config_
                        .movingObstacleMinSpeed,

                    config_
                        .movingObstacleMaxSpeed);


            // ----------------------------------------------------
            // 沿 X
            // ----------------------------------------------------

            if (
                obstacle.motion
                ==
                ObstacleMotion::Longitudinal)
            {
                const sf::Scalar
                    availableHalfRange =
                        std::max(
                            sf::Scalar(0.25),

                            (xMaximum
                             - xMinimum)
                            * 0.45);


                obstacle.range =
                    RandomScalar(
                        random,

                        std::min(
                            sf::Scalar(0.80),
                            availableHalfRange),

                        std::min(
                            config_
                                .movingObstacleMaxLongitudinalRange,

                            availableHalfRange));


                const sf::Scalar
                    anchorMinimum =
                        xMinimum
                        + obstacle.range;


                const sf::Scalar
                    anchorMaximum =
                        xMaximum
                        - obstacle.range;


                obstacle.anchor =
                    RandomScalar(
                        random,

                        anchorMinimum,

                        anchorMaximum);


                obstacle.localX =
                    RandomScalar(
                        random,

                        obstacle.anchor
                            - obstacle.range,

                        obstacle.anchor
                            + obstacle.range);


                obstacle.localY =
                    RandomScalar(
                        random,
                        yMinimum,
                        yMaximum);


                obstacle.depth =
                    RandomScalar(
                        random,
                        zMinimum,
                        zMaximum);
            }


            // ----------------------------------------------------
            // 沿 Y
            // ----------------------------------------------------

            else if (
                obstacle.motion
                ==
                ObstacleMotion::Lateral)
            {
                const sf::Scalar
                    availableHalfRange =
                        std::max(
                            sf::Scalar(0.25),

                            (yMaximum
                             - yMinimum)
                            * 0.45);


                obstacle.range =
                    RandomScalar(
                        random,

                        std::min(
                            sf::Scalar(0.70),
                            availableHalfRange),

                        std::min(
                            config_
                                .movingObstacleMaxLateralRange,

                            availableHalfRange));


                const sf::Scalar
                    anchorMinimum =
                        yMinimum
                        + obstacle.range;


                const sf::Scalar
                    anchorMaximum =
                        yMaximum
                        - obstacle.range;


                obstacle.anchor =
                    RandomScalar(
                        random,

                        anchorMinimum,

                        anchorMaximum);


                obstacle.localY =
                    RandomScalar(
                        random,

                        obstacle.anchor
                            - obstacle.range,

                        obstacle.anchor
                            + obstacle.range);


                obstacle.localX =
                    RandomScalar(
                        random,
                        xMinimum,
                        xMaximum);


                obstacle.depth =
                    RandomScalar(
                        random,
                        zMinimum,
                        zMaximum);
            }


            // ----------------------------------------------------
            // 沿 Z
            // ----------------------------------------------------

            else
            {
                const sf::Scalar
                    availableHalfRange =
                        std::max(
                            sf::Scalar(0.15),

                            (zMaximum
                             - zMinimum)
                            * 0.45);


                obstacle.range =
                    RandomScalar(
                        random,

                        std::min(
                            sf::Scalar(0.35),
                            availableHalfRange),

                        std::min(
                            config_
                                .movingObstacleMaxVerticalRange,

                            availableHalfRange));


                const sf::Scalar
                    anchorMinimum =
                        zMinimum
                        + obstacle.range;


                const sf::Scalar
                    anchorMaximum =
                        zMaximum
                        - obstacle.range;


                obstacle.anchor =
                    RandomScalar(
                        random,

                        anchorMinimum,

                        anchorMaximum);


                obstacle.depth =
                    RandomScalar(
                        random,

                        obstacle.anchor
                            - obstacle.range,

                        obstacle.anchor
                            + obstacle.range);


                obstacle.localX =
                    RandomScalar(
                        random,
                        xMinimum,
                        xMaximum);


                obstacle.localY =
                    RandomScalar(
                        random,
                        yMinimum,
                        yMaximum);
            }


            // ----------------------------------------------------
            // 初始 FishProxy 周围不要立即出现移动鱼
            // ----------------------------------------------------

            const sf::Scalar worldX =
                chunk.centerX
                + obstacle.localX;


            const sf::Scalar clearRadius =
                config_
                    .initialObstacleClearRadius;


            if (
                worldX * worldX
                + obstacle.localY
                    * obstacle.localY

                <
                clearRadius
                * clearRadius)
            {
                continue;
            }


            // ----------------------------------------------------
            // 初始位置不要直接穿进固定岩石
            // ----------------------------------------------------

            bool overlapsRock =
                false;


            const sf::Scalar
                obstacleHorizontalRadius =
                    std::sqrt(
                        obstacle.halfLength
                            * obstacle.halfLength
                        +
                        obstacle.halfWidth
                            * obstacle.halfWidth);


            for (
                const FixedObstacle&
                    rock :
                    chunk.fixedObstacles)
            {
                if (!rock.active)
                {
                    continue;
                }


                const sf::Scalar dx =
                    obstacle.localX
                    - rock.localX;


                const sf::Scalar dy =
                    obstacle.localY
                    - rock.localY;


                const sf::Scalar
                    minimumDistance =
                        obstacleHorizontalRadius
                        + rock.radius
                        + 0.15;


                if (
                    dx * dx
                    + dy * dy

                    <
                    minimumDistance
                    * minimumDistance)
                {
                    overlapsRock =
                        true;

                    break;
                }
            }


            if (overlapsRock)
            {
                continue;
            }


            obstacle.active =
                true;


            SetMovingObstaclePose(
                chunk,
                obstacle);


            placed =
                true;


            break;
        }


        if (!placed)
        {
            HideMovingObstacle(
                chunk,
                obstacle);
        }
    }
}


// ================================================================
// Moving obstacles
// ================================================================

void RiverChunkManager::UpdateMovingObstacles(
    RiverChunk& chunk,
    sf::Scalar timeStep)
{
    for (
        MovingObstacle& obstacle :
        chunk.movingObstacles)
    {
        if (!obstacle.active)
        {
            continue;
        }


        switch (obstacle.motion)
        {
        case ObstacleMotion::Longitudinal:
        {
            obstacle.localX +=
                obstacle.speed
                * timeStep;


            ReflectAtBounds(
                obstacle.localX,

                obstacle.anchor
                    - obstacle.range,

                obstacle.anchor
                    + obstacle.range,

                obstacle.speed);

            break;
        }


        case ObstacleMotion::Lateral:
        {
            obstacle.localY +=
                obstacle.speed
                * timeStep;


            ReflectAtBounds(
                obstacle.localY,

                obstacle.anchor
                    - obstacle.range,

                obstacle.anchor
                    + obstacle.range,

                obstacle.speed);

            break;
        }


        case ObstacleMotion::Vertical:
        {
            obstacle.depth +=
                obstacle.speed
                * timeStep;


            ReflectAtBounds(
                obstacle.depth,

                obstacle.anchor
                    - obstacle.range,

                obstacle.anchor
                    + obstacle.range,

                obstacle.speed);

            break;
        }
        }


        SetMovingObstaclePose(
            chunk,
            obstacle);
    }
}


void RiverChunkManager::SetMovingObstaclePose(
    RiverChunk& chunk,
    MovingObstacle& obstacle)
{
    sf::Quaternion orientation =
        sf::IQ();


    sf::Vector3 velocity =
        sf::V0();


    /*
        小鱼几何默认长轴沿 +X。

        根据运动方向旋转。
    */

    switch (obstacle.motion)
    {
    case ObstacleMotion::Longitudinal:
    {
        if (obstacle.speed < 0.0)
        {
            // +X 朝向转成 -X
            orientation =
                sf::Quaternion(
                    sf::VZ(),
                    SIMD_PI);
        }


        velocity =
            sf::Vector3(
                obstacle.speed,
                0.0,
                0.0);

        break;
    }


    case ObstacleMotion::Lateral:
    {
        const sf::Scalar angle =
            obstacle.speed >= 0.0
            ? SIMD_PI * 0.5
            : -SIMD_PI * 0.5;


        orientation =
            sf::Quaternion(
                sf::VZ(),
                angle);


        velocity =
            sf::Vector3(
                0.0,
                obstacle.speed,
                0.0);

        break;
    }


    case ObstacleMotion::Vertical:
    {
        /*
            +Z 是向下。

            box 长轴原本是 +X。
        */
        const sf::Scalar angle =
            obstacle.speed >= 0.0
            ? -SIMD_PI * 0.5
            : SIMD_PI * 0.5;


        orientation =
            sf::Quaternion(
                sf::VY(),
                angle);


        velocity =
            sf::Vector3(
                0.0,
                0.0,
                obstacle.speed);

        break;
    }
    }


    SetMovingBodyPose(
        obstacle.body,

        sf::Transform(
            orientation,

            sf::Vector3(
                chunk.centerX
                    + obstacle.localX,

                obstacle.localY,

                obstacle.depth)),

        velocity,

        sf::V0());
}


// ================================================================
// Hide pooled entities
// ================================================================

void RiverChunkManager::HideFixedObstacle(
    RiverChunk& chunk,
    FixedObstacle& obstacle)
{
    obstacle.active =
        false;


    /*
        AnimatedEntity 没有必要频繁创建/删除。

        未使用对象直接放到河床下很深的位置。
    */
    const sf::Scalar hiddenZ =
        config_.riverDepth
        + config_.maxRockHeight
        + 50.0;


    SetMovingBodyPose(
        obstacle.body,

        sf::Transform(
            sf::IQ(),

            sf::Vector3(
                chunk.centerX,
                0.0,
                hiddenZ)),

        sf::V0(),

        sf::V0());
}


void RiverChunkManager::HideMovingObstacle(
    RiverChunk& chunk,
    MovingObstacle& obstacle)
{
    obstacle.active =
        false;


    const sf::Scalar hiddenZ =
        config_.riverDepth
        + config_.maxRockHeight
        + 60.0;


    SetMovingBodyPose(
        obstacle.body,

        sf::Transform(
            sf::IQ(),

            sf::Vector3(
                chunk.centerX,
                0.0,
                hiddenZ)),

        sf::V0(),

        sf::V0());
}


// ================================================================
// Chunk recycling
// ================================================================

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


    /*
        河床、河岸搬过去。
    */
    SetChunkPose(
        rearChunk);


    /*
        这个物理对象池现在代表一个全新的逻辑 chunk。

        因此重新随机：

            石头
            高度
            安全通道
            移动鱼
            移动方向
    */
    ConfigureChunkObstacles(
        rearChunk);


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


    SetChunkPose(
        frontChunk);


    ConfigureChunkObstacles(
        frontChunk);


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


// ================================================================
// Chunk queries
// ================================================================

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

        [](
            const RiverChunk& left,
            const RiverChunk& right)
        {
            return left.centerX
                < right.centerX;
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

        [](
            const RiverChunk& left,
            const RiverChunk& right)
        {
            return left.centerX
                < right.centerX;
        });
}


sf::Scalar
RiverChunkManager::GetRearMostX() const
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

            [](
                const RiverChunk& left,
                const RiverChunk& right)
            {
                return left.centerX
                    < right.centerX;
            });


    return iterator->centerX;
}


sf::Scalar
RiverChunkManager::GetFrontMostX() const
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

            [](
                const RiverChunk& left,
                const RiverChunk& right)
            {
                return left.centerX
                    < right.centerX;
            });


    return iterator->centerX;
}
