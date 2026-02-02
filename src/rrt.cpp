#include "rrt.h"
#include <iostream>

std::vector<Point2D> RRT::findPath(const Point2D& start, const Point2D& goal) {
    // 检查起点和终点是否有效
    if (!isValidPoint(start) || !isValidPoint(goal)) {
        return {};
    }
    
    // 如果起点和终点很接近
    if (start.distance(goal) <= config_.goalTolerance) {
        return {start, goal};
    }
    
    // 初始化RRT树
    std::vector<RRTNode> tree;
    tree.emplace_back(start, -1);
    
    // 主循环
    for (int i = 0; i < config_.maxIterations; i++) {
        // 生成随机点（有一定概率选择目标点）
        Point2D randomPoint = getRandomPoint(goal);
        
        // 找到树中最近的节点
        int nearestIndex = findNearestNode(tree, randomPoint);
        Point2D nearestPoint = tree[nearestIndex].point;
        
        // 向随机点扩展
        Point2D newPoint = steer(nearestPoint, randomPoint);
        
        // 检查新点是否有效且路径无碰撞
        if (isValidPoint(newPoint) && isPathValid(nearestPoint, newPoint)) {
            // 添加新节点到树中
            tree.emplace_back(newPoint, nearestIndex);
            
            // 检查是否到达目标
            if (newPoint.distance(goal) <= config_.goalTolerance) {
                // 尝试直接连接到目标
                if (isPathValid(newPoint, goal)) {
                    tree.emplace_back(goal, static_cast<int>(tree.size() - 1));
                    return reconstructPath(tree, static_cast<int>(tree.size() - 1));
                }
            }
        }
    }
    
    // 未找到路径
    return {};
}

Point2D RRT::getRandomPoint(const Point2D& goal) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    
    // 以一定概率直接选择目标点
    if (dist(rng_) < config_.goalSampleRate) {
        return goal;
    }
    
    // 在边界内生成随机点
    std::uniform_real_distribution<double> xDist(bounds_[0], bounds_[1]);
    std::uniform_real_distribution<double> yDist(bounds_[2], bounds_[3]);
    
    return Point2D(xDist(rng_), yDist(rng_));
}

int RRT::findNearestNode(const std::vector<RRTNode>& tree, const Point2D& point) const {
    int nearestIndex = 0;
    double minDistance = tree[0].point.distance(point);
    
    for (size_t i = 1; i < tree.size(); i++) {
        double distance = tree[i].point.distance(point);
        if (distance < minDistance) {
            minDistance = distance;
            nearestIndex = static_cast<int>(i);
        }
    }
    
    return nearestIndex;
}

Point2D RRT::steer(const Point2D& from, const Point2D& to) const {
    double distance = from.distance(to);
    
    // 如果距离小于步长，直接返回目标点
    if (distance <= config_.stepSize) {
        return to;
    }
    
    // 否则按步长方向移动
    double ratio = config_.stepSize / distance;
    return from.interpolate(to, ratio);
}

bool RRT::isPathValid(const Point2D& start, const Point2D& end) const {
    // 检查线段是否与任何障碍物相交
    for (const auto& obstacle : obstacles_) {
        if (obstacle.intersects(start, end)) {
            return false;
        }
    }
    return true;
}

bool RRT::isValidPoint(const Point2D& point) const {
    // 检查点是否在边界内
    if (point.x < bounds_[0] || point.x > bounds_[1] || 
        point.y < bounds_[2] || point.y > bounds_[3]) {
        return false;
    }
    
    // 检查点是否在障碍物内
    for (const auto& obstacle : obstacles_) {
        if (obstacle.contains(point)) {
            return false;
        }
    }
    
    return true;
}

std::vector<Point2D> RRT::reconstructPath(const std::vector<RRTNode>& tree, int goalIndex) const {
    std::vector<Point2D> path;
    int currentIndex = goalIndex;
    
    // 从目标回溯到起点
    while (currentIndex != -1) {
        path.push_back(tree[currentIndex].point);
        currentIndex = tree[currentIndex].parentIndex;
    }
    
    // 反转路径，使其从起点到终点
    std::reverse(path.begin(), path.end());
    return path;
}