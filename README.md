# RRT 路径规划算法

基于 C++ 实现的快速探索随机树（Rapidly-exploring Random Tree, RRT）算法，用于 2D 环境中的路径规划和障碍物避障。

## 功能特性

- 完整的 RRT 算法实现，支持参数配置
- 障碍物碰撞检测（矩形障碍物）
- 多个测试用例，覆盖各种场景
- CMake 构建系统支持
- C++17 标准

## 目录结构

```
rrt/
├── CMakeLists.txt      # CMake 构建配置文件
├── README.md           # 本说明文件
├── .gitignore          # Git 忽略规则
├── include/            # 头文件目录
│   └── rrt.h           # RRT 核心算法声明
├── src/                # 源文件目录
│   ├── main.cpp        # 演示应用程序
│   └── rrt.cpp         # RRT 算法实现
└── tests/              # 测试文件目录
    └── test_rrt.cpp    # 完整的测试套件
```

## 构建方法

```bash
mkdir build
cd build
cmake ..
make
```

## 运行方式

### 演示应用程序
```bash
./rrt_planning
```

### 运行测试
```bash
./rrt_test
```

## 算法原理

RRT 算法的工作原理：
1. 从起始点开始构建搜索树
2. 在搜索空间中迭代采样随机点
3. 找到现有树中距离最近的节点
4. 向采样点方向以固定步长扩展树
5. 检查扩展路径是否与障碍物发生碰撞
6. 重复上述过程，直到到达目标点或达到最大迭代次数

## 配置选项

- `stepSize`：每次迭代向采样点扩展的距离
- `maxIterations`：达到最大迭代次数后停止搜索
- `goalSampleRate`：直接采样目标点的概率
- `goalTolerance`：判定到达目标点的距离阈值

## 许可证

MIT 许可证