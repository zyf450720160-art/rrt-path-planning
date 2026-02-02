# RRT Path Planning Algorithm

A C++ implementation of the Rapidly-exploring Random Tree (RRT) algorithm for 2D path planning with obstacle avoidance.

## Features

- Complete RRT implementation with configurable parameters
- Obstacle collision detection (rectangular obstacles)
- Multiple test cases covering various scenarios
- CMake build system support
- C++17 standard

## Directory Structure

```
rrt/
├── CMakeLists.txt      # CMake build configuration
├── README.md           # This file
├── .gitignore          # Git ignore rules
├── include/            # Header files
│   └── rrt.h           # RRT core algorithm declarations
├── src/                # Source files
│   ├── main.cpp        # Demo application
│   └── rrt.cpp         # RRT implementation
└── tests/              # Test files
    └── test_rrt.cpp    # Comprehensive test suite
```

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Running

### Demo Application
```bash
./rrt_planning
```

### Tests
```bash
./rrt_test
```

## Algorithm Details

The RRT algorithm works by:
1. Starting from an initial point
2. Iteratively sampling random points in the search space
3. Finding the nearest node in the existing tree
4. Extending the tree toward the sampled point by a fixed step size
5. Checking for collisions with obstacles
6. Repeating until the goal is reached or maximum iterations are exceeded

## Configuration Options

- `stepSize`: Distance to extend the tree in each iteration
- `maxIterations`: Maximum number of iterations before giving up
- `goalSampleRate`: Probability of sampling the goal point directly
- `goalTolerance`: Distance threshold to consider the goal reached

## License

MIT License