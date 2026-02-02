#include "rrt.h"
#include <iostream>
#include <vector>

int main() {
    std::cout << "RRT 路径规划算法演示" << std::endl;
    
    // 创建障碍物
    std::vector<Obstacle> obstacles = {
        Obstacle(2.0, 2.0, 2.0, 2.0),  // 中心障碍物
        Obstacle(6.0, 1.0, 1.0, 3.0),  // 右侧障碍物
        Obstacle(1.0, 6.0, 3.0, 1.0)   // 上方障碍物
    };
    
    // 配置RRT参数
    RRT::Config config;
    config.stepSize = 0.5;
    config.maxIterations = 2000;
    config.goalSampleRate = 0.1;
    config.goalTolerance = 0.3;
    
    // 创建RRT实例
    RRT rrt(obstacles, config);
    rrt.setBounds(0.0, 10.0, 0.0, 10.0);
    
    // 设置起点和终点
    Point2D start(1.0, 1.0);
    Point2D goal(9.0, 9.0);
    
    // 执行路径规划
    std::vector<Point2D> path = rrt.findPath(start, goal);
    
    if (!path.empty()) {
        std::cout << "找到包含 " << path.size() << " 个点的路径：" << std::endl;
        for (const auto& point : path) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "未找到路径！" << std::endl;
    }
    
    return 0;
}