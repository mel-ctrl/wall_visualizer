#include "wall_visualizer.hpp"
#include "lib/toml.hpp"
#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>
namespace wall_visualizer {

namespace color_codes {
// ANSI color codes
const std::string RED = "\033[91m";
const std::string GREEN = "\033[92m";
const std::string YELLOW = "\033[93m";
const std::string BLUE = "\033[94m";
const std::string MAGENTA = "\033[95m";
const std::string CYAN = "\033[96m";
const std::string ORANGE = "\033[38;5;208m";
const std::string PURPLE = "\033[38;5;129m";
const std::string PINK = "\033[38;5;213m";
const std::string GRAY = "\033[90m";
const std::string RESET = "\033[0m";

std::string GetStrideColor(int stride_id) {
  const std::vector<std::string> colors = {RED,  GREEN,  YELLOW, BLUE, MAGENTA,
                                           CYAN, ORANGE, PURPLE, PINK, GRAY};
  return colors[stride_id % colors.size()];
}
} // namespace color_codes

WallVisualizer::WallVisualizer(const std::string &config_path)
    : mTotalBricks(0), mTotalWallArea(0) {
  mConfig = LoadConfig(config_path);
  mRobotPosIdxWithMaxPlaceableBricksTmp.reserve(10 * 1024);
}

Config WallVisualizer::LoadConfig(const std::string &file_path) {
  toml::table table;
  try {
    table = toml::parse_file(file_path);
  } catch (const toml::parse_error &err) {
    throw std::runtime_error(
        std::format("[ParseGeneralConfig] Parsing failed: {}", err.what()));
  }

  Config config;
  auto getDoubleValue = [&](const std::string &section,
                            const std::string &key) -> double {
    auto section_table = table[section];
    if (!section_table) {
      throw std::runtime_error("Missing [" + section +
                               "] section in config file");
    }

    auto value_opt = section_table[key].value<double>();
    if (!value_opt) {
      throw std::runtime_error("Missing or invalid '" + key + "' in [" +
                               section + "] section");
    }

    return value_opt.value();
  };

  // Now use it for all config values
  try {
    config.wall.length = ConvertMMtoUM(getDoubleValue("WALL", "length_mm"));
    config.wall.height = ConvertMMtoUM(getDoubleValue("WALL", "height_mm"));

    config.fullBrickDimension.length =
        ConvertMMtoUM(getDoubleValue("FULL_BRICK", "length_mm"));
    config.fullBrickDimension.height =
        ConvertMMtoUM(getDoubleValue("FULL_BRICK", "height_mm"));
    config.fullBrickDimension.width =
        ConvertMMtoUM(getDoubleValue("FULL_BRICK", "width_mm"));

    config.halfBrickDimension.length =
        ConvertMMtoUM(getDoubleValue("HALF_BRICK", "length_mm"));
    config.halfBrickDimension.height =
        ConvertMMtoUM(getDoubleValue("HALF_BRICK", "height_mm"));
    config.halfBrickDimension.width =
        ConvertMMtoUM(getDoubleValue("HALF_BRICK", "width_mm"));

    config.jointSize.head =
        ConvertMMtoUM(getDoubleValue("JOINT_SIZE", "head_mm"));
    config.jointSize.bed =
        ConvertMMtoUM(getDoubleValue("JOINT_SIZE", "bed_mm"));

    config.envelope.length =
        ConvertMMtoUM(getDoubleValue("ENVELOPE", "length_mm"));
    config.envelope.height =
        ConvertMMtoUM(getDoubleValue("ENVELOPE", "height_mm"));

  } catch (const std::exception &e) {
    throw std::runtime_error(std::string("Config parsing error: ") + e.what());
  }
  config.ComputeDerivedValues();
  return config;
}

size_t WallVisualizer::ConvertMMtoUM(double value) {
  return static_cast<size_t>(std::round(value * 1000.0));
}

void WallVisualizer::CreateLayout(BondType bondType) {
  size_t nr_of_bricks_h =
      round(mConfig.envelope.length / mConfig.fullBrickDimension.length);
  switch (bondType) {
  case BondType::STRETCHER:
    CreateStretcherLayout();

    mMaxBricksAtOnce = nr_of_bricks_h * (nr_of_bricks_h + 1);
    std::cout << "max at once: " << mMaxBricksAtOnce << std::endl;

    break;
  default:
    throw std::runtime_error("Bond type not implemented");
  }
}

void WallVisualizer::CreateStretcherLayout() {
  auto nrOfCourses = mConfig.wall.height / mConfig.courseHeight;
  mWall.clear();
  mTotalBricks = 0;
  mTotalWallArea = 0;

  auto addBrick = [this](std::vector<Brick> &course, BrickType brickType,
                         auto row, auto column, auto x_pos,
                         size_t index) -> size_t {
    Coordinate coordinate(row, column);
    Position position(x_pos, row * mConfig.courseHeight);

    course.push_back(Brick(coordinate, position, brickType, 0, index));

    mTotalWallArea += (brickType == BrickType::FULL) ? mConfig.fullBrickArea
                                                     : mConfig.halfBrickArea;

    return (brickType == BrickType::HALF) ? mConfig.halfBrickWithJointLength
                                          : mConfig.fullBrickWithJointLength;
  };

  for (size_t row = 0; row < nrOfCourses; ++row) {
    std::vector<Brick> course;
    size_t currentWidth = 0;
    size_t column = 0;

    if (row % 2 == 1) {
      currentWidth += addBrick(course, BrickType::HALF, row, column,
                               currentWidth, mTotalBricks);
      mTotalBricks++;
      column++;
    }

    while (currentWidth + mConfig.fullBrickDimension.length <=
           mConfig.wall.length) {
      currentWidth += addBrick(course, BrickType::FULL, row, column,
                               currentWidth, mTotalBricks);
      mTotalBricks++;
      column++;
    }

    if (currentWidth + mConfig.halfBrickDimension.length <=
        mConfig.wall.length) {
      currentWidth += addBrick(course, BrickType::HALF, row, column,
                               currentWidth, mTotalBricks);
      mTotalBricks++;
      column++;
    }

    mWall.push_back(std::move(course));
  }

  PrecomputeSupportDependencies();
  std::cout << "\n=== BRICK LAYOUT DEBUG ===" << std::endl;
  for (size_t rowIdx = 0; rowIdx < std::min(size_t(2), mWall.size());
       ++rowIdx) {
    std::cout << "Row " << rowIdx << ":" << std::endl;
    for (const auto &brick : mWall[rowIdx]) {
      int brickEnd =
          brick.position.x + ((brick.brickType == BrickType::FULL)
                                  ? mConfig.fullBrickDimension.length
                                  : mConfig.halfBrickDimension.length);
      std::cout << "  Col " << brick.coordinate.column << ": x=["
                << brick.position.x << ", " << brickEnd << "]"
                << ", type="
                << (brick.brickType == BrickType::FULL ? "FULL" : "HALF")
                << ", index=" << brick.index << std::endl;
    }
  }
}

bool WallVisualizer::OptimizeBuildOrder() {
  std::cout << "Total robot positions: " << mAllRobotPositions.size()
            << std::endl;
  std::cout << "Total bricks: " << mTotalBricks << std::endl;
  std::cout << "Total wall area: " << mTotalWallArea << std::endl;

  mBitsetTable.clear();

  std::unordered_map<State, State, StateHash> came_from;

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>>
      priority_queue;

  std::unordered_map<State, double, StateHash> g_scores;
  g_scores.reserve(1000000);

  size_t startBitsetIdx = GetOrCreateBitsetIndex(std::bitset<MAX_BRICKS>());

  auto starting_state = State(0, startBitsetIdx, 0);
  auto &pruned_states = GetPrunedStates(starting_state);
  for (auto const &state : pruned_states) {
    Node start_node;
    start_node.cost = 0.0;
    start_node.estimatedTotalCost = CalculateHeuristic(state);
    start_node.state = state;
    start_node.currentStrideId = 0;

    g_scores[state] = start_node.cost;
    priority_queue.push(std::move(start_node));
  }

  size_t nodes_explored = 0;
  while (!priority_queue.empty()) {
    Node current_node = priority_queue.top();
    priority_queue.pop();

    nodes_explored++;

    if (nodes_explored % 1000 == 0) {
      size_t nrPlaced = current_node.state.nrOfPlacedBricks;
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Explored " << nodes_explored << ", placed " << nrPlaced
                << "/" << mTotalBricks << ", cost " << current_node.cost
                << ", f(n)=" << current_node.estimatedTotalCost
                << ", h(n)=" << CalculateHeuristic(current_node.state)
                << ", queue " << priority_queue.size() << std::endl;
    }

    // Check if we've found a better path to this state
    auto it = g_scores.find(current_node.state);
    if (it != g_scores.end() && current_node.cost > it->second) {
      continue;
    }

    // Goal check
    size_t nrOfBricksPlaced = current_node.state.nrOfPlacedBricks;
    if (nrOfBricksPlaced == mTotalBricks) {
      ReconstructPath(came_from, current_node.state);
      std::cout << "h(G): " << CalculateHeuristic(current_node.state)
                << std::endl;
      std::cout << "\n✓ Optimal solution found!" << std::endl;
      std::cout << "  Total cost: " << current_node.cost << std::endl;
      std::cout << "  Total strides: " << current_node.currentStrideId
                << std::endl;
      std::cout << "  Nodes explored: " << nodes_explored << std::endl;
      return true;
    }

    auto &pruned_states = GetPrunedStates(current_node.state);
    auto new_cost = current_node.cost + 1.0;

    for (auto const &state : pruned_states) {
      auto it = g_scores.find(state);
      if (it == g_scores.end() || new_cost < it->second) {
        mNodeTmp.cost = new_cost;
        mNodeTmp.estimatedTotalCost = new_cost + CalculateHeuristic(state);
        mNodeTmp.state = state;
        mNodeTmp.currentStrideId = current_node.currentStrideId + 1;

        g_scores[mNodeTmp.state] = mNodeTmp.cost;
        came_from[state] = current_node.state;
        priority_queue.push(std::move(mNodeTmp));
      }
    }
  }

  std::cout << "✗ No solution found!" << std::endl;
  return false;
}

std::vector<State> &WallVisualizer::GetPrunedStates(const State &state) {
  mStatesTmp.clear();
  mPrunedStatesTmp.clear();

  for (size_t i = 0; i < mAllRobotPositions.size(); ++i) {
    mStatesTmp.push_back(CreateBrickPlacementState(
        State(i, state.placedBricksIdx, state.nrOfPlacedBricks)));
  }

  return PruneStates(mStatesTmp, mPrunedStatesTmp);
}

std::vector<State> &
WallVisualizer::PruneStates(const std::vector<State> &states,
                            std::vector<State> &result) {

  size_t highest_nr_of_placed_bricks = 0;
  for (const auto &state : states) {

    if (state.nrOfPlacedBricks > 0) {
      if (state.nrOfPlacedBricks > highest_nr_of_placed_bricks) {
        // Found new max - clear old positions and start fresh
        highest_nr_of_placed_bricks = state.nrOfPlacedBricks;
        result.clear();
        result.push_back(state);
      } else if (state.nrOfPlacedBricks == highest_nr_of_placed_bricks) {
        result.push_back(state);
      }
    }
  }
  return result;
}

State WallVisualizer::CreateBrickPlacementState(const State &state) {
  const RobotPosition &robot_pos = mAllRobotPositions[state.robotPositionIdx];

  const auto &reachable_bricks = robot_pos.reachableBricks;

  std::bitset<MAX_BRICKS> current_bricks = mBitsetTable[state.placedBricksIdx];

  std::bitset<MAX_BRICKS> new_placed_bricks = current_bricks;

  auto nr_of_placed = state.nrOfPlacedBricks;

  bool placed_any = true;
  while (placed_any) {
    placed_any = false;
    for (const auto &brick : reachable_bricks) {

      // Check if already placed
      if (new_placed_bricks.test(brick.index)) {
        continue;
      }

      // Check support
      if (!IsBrickFullySupported(brick, new_placed_bricks)) {
        continue;
      }

      placed_any = true;

      new_placed_bricks.set(brick.index);
      nr_of_placed += 1;
    }
  }

  size_t newBitsetIdx = GetOrCreateBitsetIndex(new_placed_bricks);
  return State(state.robotPositionIdx, newBitsetIdx, nr_of_placed);
}

double WallVisualizer::CalculateHeuristic(const State &state) const {

  return static_cast<double>(mTotalBricks - state.nrOfPlacedBricks) /
         (mMaxBricksAtOnce + 1); // adding 1 extra brick since it is better to
                                 // underestimate the cost
}
void WallVisualizer::InitializeRobotPositions() {
  mAllRobotPositions.clear();

  int hStep = mConfig.halfBrickDimension.length;
  int vStep = mConfig.courseHeight;

  int hStart = 0;
  int hEnd = mConfig.wall.length;
  int vStart = 0;
  int vEnd = mConfig.wall.height;

  for (int hPos = hStart; hPos < hEnd; hPos += hStep) {
    for (int vPos = vStart; vPos < vEnd; vPos += vStep) {
      auto pos = Position(hPos, vPos);
      ;
      mAllRobotPositions.emplace_back(RobotPosition{
          .position = pos,
          .reachableBricks = CalculateReachableBricks(pos),
      });
    }
  }

  std::cout << "  Total positions: " << mAllRobotPositions.size() << std::endl;
}
void WallVisualizer::PrecomputeSupportDependencies() {
  for (size_t rowIdx = 0; rowIdx < mWall.size(); ++rowIdx) {
    if (rowIdx == 0) {
      // Ground level bricks have no dependencies - already empty
      continue;
    }

    auto &course = mWall[rowIdx];
    for (auto &brick : course) {
      int brickStart = brick.position.x;
      int brickLen = (brick.brickType == BrickType::FULL)
                         ? mConfig.fullBrickDimension.length
                         : mConfig.halfBrickDimension.length;
      int brickEnd = brickStart + brickLen;

      const auto &courseBelow = mWall[rowIdx - 1];

      for (const auto &brickBelow : courseBelow) {
        int belowStart = brickBelow.position.x;
        int belowLen = (brickBelow.brickType == BrickType::FULL)
                           ? mConfig.fullBrickDimension.length
                           : mConfig.halfBrickDimension.length;
        int belowEnd = belowStart + belowLen + mConfig.jointSize.head;

        int overlapStart = std::max(brickStart, belowStart);
        int overlapEnd = std::min(brickEnd, belowEnd);
        int overlap = std::max(0, overlapEnd - overlapStart);

        if (overlap > 0) {
          brick.requiredSupportIndices.push_back(brickBelow.index);
        }
      }
    }
  }
}

bool WallVisualizer::IsBrickFullySupported(
    const Brick &brick, const std::bitset<MAX_BRICKS> &placedBricksBitset) {

  // Ground level or no dependencies
  if (brick.requiredSupportIndices.empty()) {
    return true;
  }

  // ALL required bricks must be placed
  for (auto reqBrickIndex : brick.requiredSupportIndices) {
    if (!placedBricksBitset.test(reqBrickIndex)) {
      return false;
    }
  }

  return true;
}

std::vector<Brick>
WallVisualizer::CalculateReachableBricks(const Position &position) {
  std::vector<Brick> result;

  for (const auto &course : mWall) {
    for (const auto &brick : course) {
      if (CalculateBrickInReach(brick, position)) {
        result.push_back(brick);
      }
    }
  }
  return result;
}

bool WallVisualizer::CalculateBrickInReach(const Brick &brick,
                                           const Position &robotPosition) {
  int brickStart = brick.position.x;
  int brickLen = (brick.brickType == BrickType::FULL)
                     ? mConfig.fullBrickDimension.length
                     : mConfig.halfBrickDimension.length;
  int brickEnd = brickStart + brickLen;

  int brickBottomY = brick.position.y;
  int brickHeight = (brick.brickType == BrickType::FULL)
                        ? mConfig.fullBrickDimension.height
                        : mConfig.halfBrickDimension.height;
  int brickTopY = brickBottomY + brickHeight;

  int hReachStart = robotPosition.x;
  int hReachEnd = robotPosition.x + mConfig.envelope.length;
  int vReachStart = robotPosition.y;
  int vReachEnd = robotPosition.y + mConfig.envelope.height;

  bool hInReach = (brickStart >= hReachStart && brickEnd <= hReachEnd);
  bool vInReach = (brickBottomY >= vReachStart && brickTopY <= vReachEnd);

  return hInReach && vInReach;
}

// ============================================================================
// Visualization
// ============================================================================

void WallVisualizer::ClearTerminal() {
  std::cout << "\x1b[H\x1b[2J\x1b[3J" << std::flush;
}

void WallVisualizer::Visualize(
    const std::unordered_set<Coordinate, CoordinateHash> &builtBricks) {
  ClearTerminal();

  std::unordered_map<Coordinate, int, CoordinateHash> brickStrideMap;
  for (const auto &brick : mBuildOrder) {
    brickStrideMap[brick.coordinate] = brick.strideId;
  }

  for (int row = mWall.size() - 1; row >= 0; --row) {
    const auto &course = mWall[row];
    std::string line;

    for (const auto &brick : course) {
      auto it = brickStrideMap.find(brick.coordinate);
      int strideId = (it != brickStrideMap.end()) ? it->second : 0;
      std::string strideColor = color_codes::GetStrideColor(strideId);

      if (builtBricks.count(brick.coordinate)) {
        if (brick.brickType == BrickType::FULL) {
          line += strideColor + "▓▓▓▓ " + color_codes::RESET;
        } else {
          line += strideColor + "▓▓ " + color_codes::RESET;
        }
      } else {
        if (brick.brickType == BrickType::FULL) {
          line += color_codes::GRAY + "░░░░ " + color_codes::RESET;
        } else {
          line += color_codes::GRAY + "░░ " + color_codes::RESET;
        }
      }
    }

    printf("Row %2d | %s\n", row, line.c_str());
  }
}

void WallVisualizer::ReconstructPath(
    const std::unordered_map<State, State, StateHash> &came_from,
    State goalState) {

  mBuildOrder.clear();

  std::vector<State> path;
  State current = goalState;

  // Build path from goal to start
  while (came_from.find(current) != came_from.end()) {
    path.push_back(current);
    current = came_from.at(current);
  }
  path.push_back(current);

  std::reverse(path.begin(), path.end());

  int stride_id = 0;

  for (size_t i = 0; i < path.size(); ++i) {
    const State &current_state = path[i];

    // Get previous state (or empty bitset for first state)
    std::bitset<MAX_BRICKS> prev_bricks;
    if (i > 0) {
      prev_bricks = mBitsetTable[path[i - 1].placedBricksIdx];

      // Check if robot moved
      if (path[i - 1].robotPositionIdx != current_state.robotPositionIdx) {
        stride_id++;
      }
    }

    std::bitset<MAX_BRICKS> current_bricks =
        mBitsetTable[current_state.placedBricksIdx];

    for (size_t brickIdx = 0; brickIdx < mTotalBricks; ++brickIdx) {
      if (!prev_bricks.test(brickIdx) && current_bricks.test(brickIdx)) {
        // Find the brick
        for (const auto &course : mWall) {
          for (const auto &brick : course) {
            if (brick.index == brickIdx) {
              Brick buildBrick = brick;
              buildBrick.strideId = stride_id;
              mBuildOrder.push_back(buildBrick);
            }
          }
        }
      }
    }
  }
}

void WallVisualizer::RunInteractiveBuild(BondType bond_type) {
  std::cout << "Visualizing for bond type STRETCHER" << std::endl;

  CreateLayout(bond_type);
  InitializeRobotPositions();

  bool success = OptimizeBuildOrder();
  if (!success) {
    std::cout << "Failed to find valid build order!" << std::endl;
    return;
  }

  std::cout << "Press ENTER to place next brick, 'q' to quit" << std::endl;

  std::unordered_set<Coordinate, CoordinateHash> builtBricks;

  for (size_t idx = 0; idx < mBuildOrder.size(); ++idx) {
    const auto &brick = mBuildOrder[idx];

    std::cout << "\n[" << (idx + 1) << "/" << mBuildOrder.size() << "] "
              << "Place "
              << (brick.brickType == BrickType::FULL ? "FULL" : "HALF")
              << " brick at row " << brick.coordinate.row << ", col "
              << brick.coordinate.column << "? ";

    std::string userInput;
    std::getline(std::cin, userInput);

    // Trim whitespace and convert to lowercase
    userInput.erase(0, userInput.find_first_not_of(" \t\n\r"));
    userInput.erase(userInput.find_last_not_of(" \t\n\r") + 1);
    std::transform(userInput.begin(), userInput.end(), userInput.begin(),
                   ::tolower);

    if (userInput == "q") {
      std::cout << "Build stopped" << std::endl;
      break;
    } else {
      builtBricks.insert(
          Coordinate(brick.coordinate.row, brick.coordinate.column));
      Visualize(builtBricks);
    }
  }

  std::cout << "\nPress ENTER to end program";
  std::string userInput;
  std::getline(std::cin, userInput);
}

} // namespace wall_visualizer