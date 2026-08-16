#pragma once

#include <Stonefish/core/SimulationManager.h>

#include <cstdint>
#include <string>
#include <vector>


namespace sf
{
class AnimatedEntity;
class ManualTrajectory;
}


/*
    无限河流区块配置。

    Stonefish 使用 NED 坐标系：

        +X = 上游
        -X = 下游

        +Y/-Y = 河宽方向

        +Z = 向下

        Z = 0            水面
        Z = riverDepth   河床
*/
struct RiverChunkConfig
{
    // ============================================================
    // 河流几何
    // ============================================================

    sf::Scalar chunkLength = 20.0;

    sf::Scalar riverWidth = 8.0;

    sf::Scalar riverDepth = 4.0;

    sf::Scalar bedThickness = 0.50;

    sf::Scalar bankThickness = 2.0;

    sf::Scalar bankHeight = 5.0;


    // ============================================================
    // 无限区块
    // ============================================================

    // 必须为奇数，方便初始鱼处在中央区块
    int chunkCount = 7;

    // 鱼前方至少保留多少个完整区块
    int keepChunksAhead = 2;

    // 鱼后方至少保留多少个完整区块
    int keepChunksBehind = 2;


    // ============================================================
    // 固定障碍物 —— 河底石头
    // ============================================================

    int minFixedObstaclesPerChunk = 1;

    int maxFixedObstaclesPerChunk = 3;

    // 石头很窄，防止完全堵住河道
    sf::Scalar minRockRadius = 0.20;

    sf::Scalar maxRockRadius = 0.65;

    // 普通石头露出河床的最小高度
    sf::Scalar minRockExposedHeight = 0.25;

    // 普通水下石头最大高度
    sf::Scalar maxSubmergedRockExposedHeight = 1.80;

    /*
        所有石头实际都使用同样高度的 cylinder，
        通过把多余部分埋进河床来控制露出高度。

        riverDepth = 4
        maxRockHeight = 5.2

        因此最高石头顶部可以到：

            Z = 4 - 5.2 = -1.2 m

        即高出水面约 1.2 m。
    */
    sf::Scalar maxRockHeight = 5.20;

    // 每块石头成为“出水巨石”的概率
    sf::Scalar surfacePiercingRockProbability = 0.12;


    // ============================================================
    // 河道安全性
    // ============================================================

    /*
        每个 chunk 永远保留一条无固定石头的通道。

        这可以防止随机生成出 RL 智能体根本无法通过的地图。
    */
    sf::Scalar safeCorridorWidth = 2.0;

    // 障碍物距离河岸的最小距离
    sf::Scalar obstacleBankClearance = 0.30;

    // 障碍物距离 chunk 接缝的距离
    sf::Scalar obstacleChunkEdgeClearance = 1.0;

    // 初始 FishProxy 周围不放置障碍物
    sf::Scalar initialObstacleClearRadius = 3.0;


    // ============================================================
    // 移动障碍物 —— 其他小鱼
    // ============================================================

    int minMovingObstaclesPerChunk = 0;

    int maxMovingObstaclesPerChunk = 2;

    sf::Scalar movingFishMinLength = 0.70;

    sf::Scalar movingFishMaxLength = 1.20;

    sf::Scalar movingFishMinWidth = 0.24;

    sf::Scalar movingFishMaxWidth = 0.42;

    // 高度 = width * 此系数
    sf::Scalar movingFishHeightRatio = 0.70;

    // 其他鱼的游动速度
    sf::Scalar movingObstacleMinSpeed = 0.30;

    sf::Scalar movingObstacleMaxSpeed = 1.20;

    // 沿 X 的最大往返距离的一半
    sf::Scalar movingObstacleMaxLongitudinalRange = 4.0;

    // 沿 Y 的最大往返距离的一半
    sf::Scalar movingObstacleMaxLateralRange = 3.0;

    // 沿 Z 的最大往返距离的一半
    sf::Scalar movingObstacleMaxVerticalRange = 1.20;

    // 小鱼离水面和河床至少留出的距离
    sf::Scalar movingObstacleVerticalClearance = 0.25;


    // ============================================================
    // 随机种子
    // ============================================================

    /*
        同一个 seed 下，逻辑 chunk 编号相同，
        障碍物布局也是确定性的。

        后面做 RL episode 时可以把它变成 episode seed。
    */
    std::uint64_t obstacleSeed = 20260815ULL;
};


class RiverChunkManager final
{
public:

    explicit RiverChunkManager(
        RiverChunkConfig config = {});


    /*
        创建固定数量的河流区块和障碍物对象池。
    */
    void Build(
        sf::SimulationManager& simulation);


    /*
        根据主鱼位置：

        1. 回收河流区块
        2. 更新所有移动障碍物
    */
    void Update(
        sf::Scalar fishX,
        sf::Scalar timeStep);


private:

    // ============================================================
    // 通用可移动实体
    // ============================================================

    struct MovingBody
    {
        sf::AnimatedEntity* entity = nullptr;

        sf::ManualTrajectory* trajectory = nullptr;
    };


    // ============================================================
    // 固定障碍物
    // ============================================================

    struct FixedObstacle
    {
        MovingBody body;

        bool active = false;

        sf::Scalar radius = 0.0;

        // 相对于 chunk.centerX
        sf::Scalar localX = 0.0;

        sf::Scalar localY = 0.0;

        // 从河床向上露出的高度
        sf::Scalar exposedHeight = 0.0;
    };


    // ============================================================
    // 移动障碍物
    // ============================================================

    enum class ObstacleMotion
    {
        // +X / -X，上游 / 下游
        Longitudinal,

        // +Y / -Y，左右横穿
        Lateral,

        // +Z / -Z，下潜 / 上浮
        Vertical
    };


    struct MovingObstacle
    {
        MovingBody body;

        bool active = false;

        ObstacleMotion motion =
            ObstacleMotion::Longitudinal;


        // 实体尺寸的一半
        sf::Scalar halfLength = 0.0;

        sf::Scalar halfWidth = 0.0;

        sf::Scalar halfHeight = 0.0;


        // 当前局部位置
        sf::Scalar localX = 0.0;

        sf::Scalar localY = 0.0;

        // NED 深度
        sf::Scalar depth = 2.0;


        /*
            运动轴上的中心位置。

            Longitudinal:
                anchor = local X

            Lateral:
                anchor = Y

            Vertical:
                anchor = depth
        */
        sf::Scalar anchor = 0.0;

        // anchor ± range
        sf::Scalar range = 0.0;

        /*
            带符号速度。

            Longitudinal:
                + = 上游
                - = 下游

            Lateral:
                +Y / -Y

            Vertical:
                +Z = 下潜
                -Z = 上浮
        */
        sf::Scalar speed = 0.0;
    };


    // ============================================================
    // River chunk
    // ============================================================

    struct RiverChunk
    {
        // 无限河流中的逻辑编号
        std::int64_t globalIndex = 0;

        // 当前真实世界中心 X
        sf::Scalar centerX = 0.0;

        MovingBody bed;

        MovingBody positiveYBank;

        MovingBody negativeYBank;


        // 本 chunk 永久无固定障碍物的通道中心
        sf::Scalar safeCorridorCenterY = 0.0;


        std::vector<FixedObstacle>
            fixedObstacles;

        std::vector<MovingObstacle>
            movingObstacles;
    };


    // ============================================================
    // Entity 创建
    // ============================================================

    MovingBody CreateBox(
        const std::string& name,
        const sf::Vector3& dimensions,
        const std::string& material,
        const std::string& look,
        const sf::Transform& initialTransform);


    MovingBody CreateCylinder(
        const std::string& name,
        sf::Scalar radius,
        sf::Scalar height,
        const std::string& material,
        const std::string& look,
        const sf::Transform& initialTransform);


    // ============================================================
    // Pose
    // ============================================================

    void SetChunkPose(
        RiverChunk& chunk);


    void SetMovingBodyPose(
        MovingBody& body,
        const sf::Transform& transform,
        const sf::Vector3& linearVelocity,
        const sf::Vector3& angularVelocity);


    // ============================================================
    // 障碍物
    // ============================================================

    void ConfigureChunkObstacles(
        RiverChunk& chunk);


    void UpdateMovingObstacles(
        RiverChunk& chunk,
        sf::Scalar timeStep);


    void SetMovingObstaclePose(
        RiverChunk& chunk,
        MovingObstacle& obstacle);


    void HideFixedObstacle(
        RiverChunk& chunk,
        FixedObstacle& obstacle);


    void HideMovingObstacle(
        RiverChunk& chunk,
        MovingObstacle& obstacle);


    std::uint64_t MakeChunkSeed(
        std::int64_t globalIndex) const;


    // ============================================================
    // 无限 chunk 回收
    // ============================================================

    void RecycleRearChunkToFront();

    void RecycleFrontChunkToRear();


    RiverChunk& GetRearMostChunk();

    RiverChunk& GetFrontMostChunk();


    sf::Scalar GetRearMostX() const;

    sf::Scalar GetFrontMostX() const;


    // ============================================================
    // 数据
    // ============================================================

    RiverChunkConfig config_;

    sf::SimulationManager* simulation_ =
        nullptr;

    std::vector<RiverChunk> chunks_;


    // 新生成到 +X 时使用
    std::int64_t nextFrontGlobalIndex_ = 0;

    // 新生成到 -X 时使用
    std::int64_t nextRearGlobalIndex_ = 0;
};
