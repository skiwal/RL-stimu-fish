#include <Stonefish/core/GraphicalSimulationApp.h>

#include <string>

#include "inc/rl_fish_simulator.h"

int main(int argc, char** argv)
{
    // Stonefish 默认渲染设置
    sf::RenderSettings renderSettings;

    // Stonefish 默认辅助显示设置
    sf::HelperSettings helperSettings;

    // 每秒执行 500 个物理仿真步
    RLFishSimulator simulator(500.0);

    sf::GraphicalSimulationApp app(
        "RL Bionic Fish Simulator",
        "data/",
        renderSettings,
        helperSettings,
        &simulator
    );

    app.Run();

    return 0;
}
