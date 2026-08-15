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

struct RiverChunkConfig
{
    // 河流几何尺寸
    sf::Scalar chunkLength = 20.0;
    sf::Scalar riverWidth = 8.0;
    sf::Scalar riverDepth = 4.0;

    sf::Scalar bedThickness = 0.50;

    sf::Scalar bankThickness = 2.0;
    sf::Scalar bankHeight = 5.0;

    // 必须是奇数，方便鱼初始位于中央区块
    int chunkCount = 7;

    // 鱼前方至少保留多少个完整区块
    int keepChunksAhead = 2;

    // 鱼后方至少保留多少个完整区块
    int keepChunksBehind = 2;
};

class RiverChunkManager final
{
public:
    explicit RiverChunkManager(
        RiverChunkConfig config = {});

    // 创建固定数量的河流区块
    void Build(sf::SimulationManager& simulation);

    // 根据鱼的世界坐标更新区块
    void Update(sf::Scalar fishX);

private:
    struct MovingBox
    {
        sf::AnimatedEntity* entity = nullptr;
        sf::ManualTrajectory* trajectory = nullptr;
    };

    struct RiverChunk
    {
        // 无限河流中的逻辑编号
        std::int64_t globalIndex = 0;

        // 当前 Stonefish 局部世界中的中心位置
        sf::Scalar centerX = 0.0;

        MovingBox bed;
        MovingBox positiveYBank;
        MovingBox negativeYBank;
    };

    MovingBox CreateBox(
        const std::string& name,
        const sf::Vector3& dimensions,
        const std::string& look,
        const sf::Transform& initialTransform);

    void SetChunkPose(RiverChunk& chunk);

    void SetBoxPose(
        MovingBox& box,
        sf::Scalar x,
        sf::Scalar y,
        sf::Scalar z);

    void RecycleRearChunkToFront();
    void RecycleFrontChunkToRear();

    RiverChunk& GetRearMostChunk();
    RiverChunk& GetFrontMostChunk();

    sf::Scalar GetRearMostX() const;
    sf::Scalar GetFrontMostX() const;

    RiverChunkConfig config_;

    sf::SimulationManager* simulation_ = nullptr;

    std::vector<RiverChunk> chunks_;

    // 新生成到前方时使用
    std::int64_t nextFrontGlobalIndex_ = 0;

    // 新生成到后方时使用
    std::int64_t nextRearGlobalIndex_ = 0;
};
