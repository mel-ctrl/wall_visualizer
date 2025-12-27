# Wall Visualizer
## Overview

This program optimizes the build sequence for a robotic brick-laying system. It uses A* pathfinding to determine the optimal order and robot positions for placing bricks while respecting structural constraints; bricks must be fully supported from below.

A movement has cost 1 and placing a brick from current position has cost 0. With A* this unfolds to greedy brick placement and finding a path where you maximize amount of bricks you place while minimizing robot movements. This is because A* selects the path that minimizes: $$f(n) = g(n) + h(n)$$ It tries to greedily reduce total cost $f(n)$, so building as many bricks from current position ($g(n) = 0$ for placing) + a heuristic $h(n)$; a lower bound for the estimation of total cost remaining. So it prioritizes visiting states that are not increasing the total cost. A very low bound for the heuristic would be area to build / envelope area. 

The heuristic $h(n)$ is constant for each state with an added brick in the middle of the wall (does not matter if you place a brick at position A or position B, heuristic is the same if the added brick
is same size which is the case for the middle part of a wall with a brick pattern of same sized bricks). Since there were too many states (a state consists of robot position and wall state) with equally good estimated remaining costs, keeping it pure A* would take too much time to run the program. Especially for wide walls it would take too long since in the height you can remove lots of positions where you can not build any brick because there is no support yet.

That is why I made the code such that brick placement is always greedy. A* tries to find the path which will maximize the most bricks built in least movements as possible. The robot positions are pruned to speed things up.

#### Key Parameter

##### `top_x_brick_count_to_keep`
This parameter is used for the pruning strategy. 
**What it does:** Keeps only robot positions that can place the top X distinct brick counts.
**Example:** `top_x_brick_count_to_keep = 3`
  - Position A can place 10 bricks (most)
  - Position B can place 10 bricks (most) 
  - Position C can place 8 bricks (second most)
  - Position D can place 5 bricks
  Result: Keep A, B, and C (all positions with top 2 distinct counts: 10 and 8)

**Tuning guidance:**
- **Lower values (1-2)**: Faster execution, more aggressive pruning, may miss optimal solution
- **Higher values (5-10)**: Slower execution, explores more possibilities, better solution quality
- **Value >= total bricks**: Disables pruning, you will find the global optimum but may take long to run.
- **Recommended starting point**: 2 for most walls.

# How to run
## 1. Cloning the repo
`git@github.com:mel-ctrl/wall_visualizer.git`

## 2. Building the Program

### Prerequisites
- C++20 compatible compiler (GCC 10+ or Clang 10+)
- CMake 3.15 or higher

### Build with CMake (Recommended)
Go to the cloned folder. Run from command line: 
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

## 3. Running the program
From within the build folder use: 
```bash
# Run with stretcher bond
./wall_visualizer --config ../config.toml --bond_type stretcher

# Run with English cross bond
./wall_visualizer --config ../config.toml --bond_type english_cross_bond
```

### Config file
The program requires a TOML configuration file specifying wall dimensions, brick sizes, robot envelope, and optimization parameters.

#### Configuration File Format
```toml
[PARAMETERS]
top_x_brick_count_to_keep = 2

[WALL]
length_mm = 2300
height_mm = 2000

[FULL_BRICK]
length_mm = 210
height_mm = 50
width_mm = 100

[HALF_BRICK]
length_mm = 100
height_mm = 50
width_mm = 100

[JOINT_SIZE]
head_mm = 10
bed_mm = 12.5

[ENVELOPE]
length_mm = 800
height_mm = 1300

```
