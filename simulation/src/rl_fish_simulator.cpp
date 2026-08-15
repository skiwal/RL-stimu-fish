#include "inc/rl_fish_simulator.h"

#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/entities/Entity.h>
#include <Stonefish/entities/SolidEntity.h>

#include <stdexcept>
#include <string>

namespace
{

RiverChunkConfig CreateRiverConfig()
{
    RiverChunkConfig config;

    config.chunkLength = 20.0;

    config.riverWidth = 8.0;
    config.riverDepth = 4.0;

    config.bedThickness = 0.50;

    config.bankThickness = 2.0;
    config.bankHeight = 5.0;

    config.chunkCount = 7;

    config.keepChunksAhead = 2;
    config.keepChunksBehind = 2;

    return config;
}

} // namespace

RLFishSimulator::RLFishSimulator(
    sf::Scalar stepsPerSecond)
    : sf::SimulationManager(stepsPerSecond),
      river_(CreateRiverConfig())
{
    /*
        必须启用，Stonefish 才会在每个物理步之后
        调用 SimulationStepCompleted()。
    */
    setCallSimulationStepCompleted(true);
}

void RLFishSimulator::BuildScenario()
{
    sf::SimulationApp* app =
        sf::SimulationApp::getApp();

    if (app == nullptr)
    {
        throw std::runtime_error(
            "Stonefish SimulationApp is not available.");
    }

    const std::string scenarioPath =
        app->getDataPath()
        + "env/basic_river.scn";

    sf::ScenarioParser parser(this);

    if (!parser.Parse(scenarioPath))
    {
        throw std::runtime_error(
            "Failed to parse scenario: "
            + scenarioPath);
    }

    /*
        SCN 解析完成后，通过唯一名称寻找临时鱼体。
    */
    sf::Entity* fishEntity =
        getEntity("FishProxy");

    fish_ =
        dynamic_cast<sf::SolidEntity*>(
            fishEntity);

    if (fish_ == nullptr)
    {
        throw std::runtime_error(
            "FishProxy was not found or is not a SolidEntity.");
    }

    /*
        创建固定数量的河床和河岸区块。
    */
    river_.Build(*this);
}

void RLFishSimulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    (void)timeStep;

    if (fish_ == nullptr)
    {
        return;
    }

    /*
        FishProxy 是正常动态刚体。

        它的位置由 Stonefish 的：
        - 刚体动力学
        - 浮力
        - 水流阻力
        - 碰撞

        共同决定。
    */
    const sf::Transform fishTransform =
        fish_->getCGTransform();

    const sf::Vector3 fishPosition =
        fishTransform.getOrigin();

    /*
        区块只根据鱼的位置进行回收。
        河床平时不会跟着鱼持续运动。
    */
    river_.Update(
        fishPosition.getX());
}
