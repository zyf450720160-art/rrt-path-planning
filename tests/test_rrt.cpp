#include "rrt.h"
#include <iostream>
#include <vector>
#include <cassert>

// 打印路径
void printPath(const std::vector<Point2D>& path) {
    if (path.empty()) {
        std::cout << "未找到路径！" << std::endl;
        return;
    }
    
    std::cout << "路径 (" << path.size() << " 个点): ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "(" << path[i].x << "," << path[i].y << ")";
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
}

// 测试用例1：简单路径规划
void testSimplePath() {
    std::cout << "\n=== 测试用例1：简单路径规划 ===" << std::endl;
    
    // 创建障碍物
    std::vector<Obstacle> obstacles = {
        Obstacle(2.0, 2.0, 2.0, 2.0),  // 中心障碍物
        Obstacle(6.0, 1.0, 1.0, 3.0)   // 右侧障碍物
    };
    
    RRT::Config config;
    config.stepSize = 0.5;
    config.maxIterations = 2000;
    config.goalSampleRate = 0.1;
    config.goalTolerance = 0.3;
    
    RRT rrt(obstacles, config);
    rrt.setBounds(0.0, 10.0, 0.0, 10.0);
    
    Point2D start(1.0, 1.0);
    Point2D goal(9.0, 9.0);
    
    std::vector<Point2D> path = rrt.findPath(start, goal);
    
    std::cout << "起点: (" << start.x << "," << start.y << ")" << std::endl;
    std::cout << "终点: (" << goal.x << "," << goal.y << ")" << std::endl;
    printPath(path);
    
    // 验证路径存在
    assert(!path.empty());
    assert(std::abs(path.front().x - start.x) < 0.1 && std::abs(path.front().y - start.y) < 0.1);
    assert(std::abs(path.back().x - goal.x) < config.goalTolerance && 
           std::abs(path.back().y - goal.y) < config.goalTolerance);
    std::cout << "✅ 测试用例1通过！" << std::endl;
}

// 测试用例2：无路径情况
void testNoPath() {
    std::cout << "\n=== 测试用例2：无路径情况 ===" << std::endl;
    
    // 创建完全阻塞的障碍物
    std::vector<Obstacle> obstacles = {
        Obstacle(0.0, 4.0, 10.0, 2.0)  // 横向墙壁
    };
    
    RRT::Config config;
    config.stepSize = 0.5;
    config.maxIterations = 1000;
    config.goalTolerance = 0.3;
    
    RRT rrt(obstacles, config);
    rrt.setBounds(0.0, 10.0, 0.0, 10.0);
    
    Point2D start(1.0, 1.0);
    Point2D goal(1.0, 9.0);
    
    std::vector<Point2D> path = rrt.findPath(start, goal);
    
    std::cout << "起点: (" << start.x << "," << start.y << ")" << std::endl;
    std::cout << "终点: (" << goal.x << "," << goal.y << ")" << std::endl;
    printPath(path);
    
    // 验证没有找到路径
    assert(path.empty());
    std::cout << "✅ 测试用例2通过！" << std::endl;
}

// 测试用例3：起点等于终点
void testSameStartGoal() {
    std::cout << "\n=== 测试用例3：起点等于终点 ===" << std::endl;
    
    std::vector<Obstacle> obstacles;
    RRT rrt(obstacles);
    rrt.setBounds(0.0, 10.0, 0.0, 10.0);
    
    Point2D start(5.0, 5.0);
    Point2D goal(5.0, 5.0);
    
    std::vector<Point2D> path = rrt.findPath(start, goal);
    
    std::cout << "起点/终点: (" << start.x << "," << start.y << ")" << std::endl;
    printPath(path);
    
    // 验证路径只包含两个点（起点和终点）
    assert(path.size() == 2);
    assert(std::abs(path[0].x - start.x) < 0.1 && std::abs(path[0].y - start.y) < 0.1);
    assert(std::abs(path[1].x - goal.x) < 0.1 && std::abs(path[1].y - goal.y) < 0.1);
    std::cout << "✅ 测试用例3通过！" << std::endl;
}

// 测试用例4：不同参数配置
void testDifferentConfigs() {
    std::cout << "\n=== 测试用例4：不同参数配置 ===" << std::endl;
    
    std::vector<Obstacle> obstacles = {
        Obstacle(3.0, 3.0, 4.0, 4.0)  // 大障碍物
    };
    
    Point2D start(1.0, 1.0);
    Point2D goal(9.0, 9.0);
    
    // 测试不同的步长
    RRT::Config config1;
    config1.stepSize = 0.3;
    config1.maxIterations = 3000;
    config1.goalTolerance = 0.2;
    
    RRT rrt1(obstacles, config1);
    rrt1.setBounds(0.0, 10.0, 0.0, 10.0);
    std::vector<Point2D> path1 = rrt1.findPath(start, goal);
    
    RRT::Config config2;
    config2.stepSize = 1.0;
    config2.maxIterations = 1000;
    config2.goalTolerance = 0.5;
    
    RRT rrt2(obstacles, config2);
    rrt2.setBounds(0.0, 10.0, 0.0, 10.0);
    std::vector<Point2D> path2 = rrt2.findPath(start, goal);
    
    std::cout << "小步长路径长度: " << path1.size() << std::endl;
    std::cout << "大步长路径长度: " << path2.size() << std::endl;
    
    // 验证两种配置都能找到路径
    assert(!path1.empty());
    assert(!path2.empty());
    std::cout << "✅ 测试用例4通过！" << std::endl;
}

// 测试用例5：复杂环境
void testComplexEnvironment() {
    std::cout << "\n=== 测试用例5：复杂环境 ===" << std::endl;
    
    // 创建多个障碍物
    std::vector<Obstacle> obstacles = {
        Obstacle(1.0, 1.0, 1.0, 1.0),
        Obstacle(3.0, 2.0, 1.0, 2.0),
        Obstacle(5.0, 1.0, 1.0, 1.0),
        Obstacle(7.0, 3.0, 1.0, 1.0),
        Obstacle(2.0, 6.0, 2.0, 1.0),
        Obstacle(6.0, 5.0, 1.0, 2.0)
    };
    
    RRT::Config config;
    config.stepSize = 0.4;
    config.maxIterations = 5000;
    config.goalSampleRate = 0.15;
    config.goalTolerance = 0.25;
    
    RRT rrt(obstacles, config);
    rrt.setBounds(0.0, 10.0, 0.0, 10.0);
    
    Point2D start(0.5, 0.5);
    Point2D goal(9.5, 9.5);
    
    std::vector<Point2D> path = rrt.findPath(start, goal);
    
    std::cout << "复杂环境路径长度: " << path.size() << std::endl;
    printPath({path.front(), path.back()}); // 只打印起点和终点
    
    // 验证路径存在
    assert(!path.empty());
    assert(std::abs(path.front().x - start.x) < 0.1 && std::abs(path.front().y - start.y) < 0.1);
    assert(std::abs(path.back().x - goal.x) < config.goalTolerance && 
           std::abs(path.back().y - goal.y) < config.goalTolerance);
    std::cout << "✅ 测试用例5通过！" << std::endl;
}

int main() {
    std::cout << "正在运行RRT算法测试..." << std::endl;
    
    try {
        testSimplePath();
        testNoPath();
        testSameStartGoal();
        testDifferentConfigs();
        testComplexEnvironment();
        
        std::cout << "\n🎉 所有RRT测试均成功通过！" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "测试失败，异常信息: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "测试失败，未知异常" << std::endl;
        return 1;
    }
}