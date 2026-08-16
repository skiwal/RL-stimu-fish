#include "inc/rl_fish_simulator.h"

#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/Entity.h>
#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/entities/forcefields/Ocean.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <stdexcept>
#include <string>


namespace
{

RiverChunkConfig CreateRiverConfig()
{
    RiverChunkConfig config;


    // ============================================================
    // 河道
    // ============================================================

    config.chunkLength =
        20.0;

    config.riverWidth =
        8.0;

    config.riverDepth =
        4.0;

    config.bedThickness =
        0.50;

    config.bankThickness =
        2.0;

    config.bankHeight =
        5.0;


    // ============================================================
    // 无限河道
    // ============================================================

    config.chunkCount =
        7;

    config.keepChunksAhead =
        2;

    config.keepChunksBehind =
        2;


    // ============================================================
    // 固定障碍物
    // ============================================================

    config.minFixedObstaclesPerChunk =
        2;

    config.maxFixedObstaclesPerChunk =
        5;


    config.minRockRadius =
        0.20;

    config.maxRockRadius =
        0.65;


    config.minRockExposedHeight =
        0.25;

    config.maxSubmergedRockExposedHeight =
        1.80;


    /*
        河深 4 m。

        5.2 m 的 exposed height
        可以让顶部达到 Z=-1.2。
    */
    config.maxRockHeight =
        5.20;


    /*
        大约 12% 石头会成为出水巨石。
    */
    config.surfacePiercingRockProbability =
        0.12;


    // ============================================================
    // 保证可通行
    // ============================================================

    config.safeCorridorWidth =
        2.0;

    config.obstacleBankClearance =
        0.30;

    config.obstacleChunkEdgeClearance =
        1.0;


    /*
        开局 FishProxy 附近保持干净。
    */
    config.initialObstacleClearRadius =
        3.0;


    // ============================================================
    // 移动障碍物
    // ============================================================

    config.minMovingObstaclesPerChunk =
        0;

    config.maxMovingObstaclesPerChunk =
        2;


    config.movingFishMinLength =
        0.70;

    config.movingFishMaxLength =
        1.20;


    config.movingFishMinWidth =
        0.24;

    config.movingFishMaxWidth =
        0.42;


    config.movingFishHeightRatio =
        0.70;


    config.movingObstacleMinSpeed =
        0.30;

    config.movingObstacleMaxSpeed =
        1.20;


    config.movingObstacleMaxLongitudinalRange =
        4.0;

    config.movingObstacleMaxLateralRange =
        3.0;

    config.movingObstacleMaxVerticalRange =
        1.20;


    config.movingObstacleVerticalClearance =
        0.25;


    // ============================================================
    // 随机种子
    // ============================================================

    config.obstacleSeed =
        20260815ULL;


    return config;
}

} // namespace


RLFishSimulator::RLFishSimulator(
    sf::Scalar stepsPerSecond)
    : sf::SimulationManager(
          stepsPerSecond),
      river_(
          CreateRiverConfig())
{
    /*
        Stonefish 默认不会主动调用
        SimulationStepCompleted()。

        我们需要它来：

            - 回收河道
            - 更新移动障碍物
    */
    setCallSimulationStepCompleted(
        true);
}


void RLFishSimulator::BuildScenario()
{
    // ============================================================
    // SimulationApp
    // ============================================================

    sf::SimulationApp* app =
        sf::SimulationApp::getApp();


    if (app == nullptr)
    {
        throw std::runtime_error(
            "Stonefish SimulationApp is not available.");
    }


    // ============================================================
    // Scene
    // ============================================================

    const std::string scenarioPath =
        app->getDataPath()
        + "env/basic_river.scn";


    sf::ScenarioParser parser(
        this);


    if (!parser.Parse(
            scenarioPath))
    {
        throw std::runtime_error(
            "Failed to parse scenario: "
            + scenarioPath);
    }


    // ============================================================
    // Ocean currents
    // ============================================================

    /*
        ScenarioParser 会创建 XML 中定义的 velocity field，
        但 Ocean 的 currents 总开关必须显式开启。
    */
    sf::Ocean* ocean =
        getOcean();


    if (ocean == nullptr)
    {
        throw std::runtime_error(
            "Ocean was not created by the scenario.");
    }


    ocean->EnableCurrents();


    // ============================================================
    // Main fish
    // ============================================================

    sf::Entity* fishEntity =
        getEntity(
            "FishProxy");


    fish_ =
        dynamic_cast<
            sf::SolidEntity*>(
                fishEntity);


    if (fish_ == nullptr)
    {
        throw std::runtime_error(
            "FishProxy was not found "
            "or is not a SolidEntity.");
    }


    // ============================================================
    // Camera follows FishProxy
    // ============================================================

    /*
        Trackball 的中心始终跟随 FishProxy。

        仍然可以：

            - 鼠标围绕鱼旋转
            - 缩放
            - 改变观察角度

        但是鱼不会因为无限河道而跑出视野。
    */
    sf::OpenGLTrackball* trackball =
        getTrackball();


    if (trackball != nullptr)
    {
        trackball->GlueToMoving(
            fish_);
    }


    // ============================================================
    // Infinite river
    // ============================================================

    /*
        创建：

            河床
            河岸
            固定岩石池
            移动鱼池
    */
    river_.Build(
        *this);
}


void RLFishSimulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    if (fish_ == nullptr)
    {
        return;
    }


    /*
        FishProxy 是正常动态刚体。

        位置受：

            - gravity
            - buoyancy
            - hydrodynamics
            - ocean current
            - collision

        共同影响。
    */
    const sf::Transform fishTransform =
        fish_->getCGTransform();


    const sf::Vector3 fishPosition =
        fishTransform.getOrigin();


    /*
        同一个 Stonefish physics timeStep：

            1. 检查河道回收
            2. 更新 scripted moving obstacles
    */
    river_.Update(
        fishPosition.getX(),
        timeStep);
}
