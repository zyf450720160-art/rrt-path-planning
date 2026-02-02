#ifndef RRT_H
#define RRT_H

#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>

// 2D 点结构体
struct Point2D {
    double x, y;
    
    Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    
    // 计算到另一个点的距离
    double distance(const Point2D& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
    
    // 线性插值
    Point2D interpolate(const Point2D& other, double ratio) const {
        return Point2D(x + ratio * (other.x - x), y + ratio * (other.y - y));
    }
};

// 障碍物结构体（矩形）
struct Obstacle {
    double x, y, width, height;
    
    Obstacle(double x = 0.0, double y = 0.0, double width = 0.0, double height = 0.0)
        : x(x), y(y), width(width), height(height) {}
    
    // 检查点是否在障碍物内
    bool contains(const Point2D& point) const {
        return point.x >= x && point.x <= x + width && 
               point.y >= y && point.y <= y + height;
    }
    
    // 检查线段是否与障碍物相交
    bool intersects(const Point2D& start, const Point2D& end) const {
        // 简单的边界框检查
        if (std::max(start.x, end.x) < x || std::min(start.x, end.x) > x + width ||
            std::max(start.y, end.y) < y || std::min(start.y, end.y) > y + height) {
            return false;
        }
        
        // 更精确的线段-矩形相交检测
        // 检查线段是否完全在矩形内部
        if (contains(start) && contains(end)) {
            return true;
        }
        
        // 检查矩形是否完全在线段的一侧
        return true; // 简化实现
    }
};

// RRT 节点结构体
struct RRTNode {
    Point2D point;
    int parentIndex; // 父节点在树中的索引
    
    RRTNode(const Point2D& p, int parent = -1) : point(p), parentIndex(parent) {}
};

// RRT 路径规划器
class RRT {
public:
    struct Config {
        double stepSize;
        int maxIterations;
        double goalSampleRate;
        double goalTolerance;
        
        Config() : stepSize(1.0), maxIterations(1000), 
                   goalSampleRate(0.1), goalTolerance(0.5) {}
    };
    
    RRT(const std::vector<Obstacle>& obstacles, const Config& config = Config())
        : obstacles_(obstacles), config_(config) {
        // 初始化随机数生成器
        rng_ = std::mt19937(std::random_device{}());
    }
    
    // 设置搜索边界
    void setBounds(double minX, double maxX, double minY, double maxY) {
        bounds_ = {minX, maxX, minY, maxY};
    }
    
    // 执行RRT路径规划
    std::vector<Point2D> findPath(const Point2D& start, const Point2D& goal);
    
private:
    std::vector<Obstacle> obstacles_;
    Config config_;
    std::vector<double> bounds_; // {minX, maxX, minY, maxY}
    std::mt19937 rng_;
    
    // 生成随机点（非const，因为需要修改rng_状态）
    Point2D getRandomPoint(const Point2D& goal);
    
    // 找到树中最近的节点
    int findNearestNode(const std::vector<RRTNode>& tree, const Point2D& point) const;
    
    // 从最近节点向目标点扩展
    Point2D steer(const Point2D& from, const Point2D& to) const;
    
    // 检查路径是否无碰撞
    bool isPathValid(const Point2D& start, const Point2D& end) const;
    
    // 重建路径
    std::vector<Point2D> reconstructPath(const std::vector<RRTNode>& tree, int goalIndex) const;
    
    // 检查点是否有效（不在障碍物内）
    bool isValidPoint(const Point2D& point) const;
};

#endif // RRT_H