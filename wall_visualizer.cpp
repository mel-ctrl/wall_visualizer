#include "wall_visualizer.hpp"
#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wall_visualizer {

WallVisualizer::WallVisualizer(const std::string &config_path)
    : mWall(), mBuildOrder(), mStatesTmp(), mPrunedStatesTmp(),
      mAllRobotPositions(), mBitsetTable(), mBitsetToIndex(), mTotalBricks(0),
      mTotalWallArea(0) {
  mConfig = LoadConfig(config_path);
  mTopCountsTmp.reserve(MAX_BRICKS);
  mStatesTmp.reserve(100 * MAX_BRICKS);
  mPrunedStatesTmp.reserve(100 * MAX_BRICKS);
  mBitsetToIndex.reserve(100 * MAX_BRICKS);
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

  try {
    config.topXBrickCount = GetConfigValue<size_t>(table, "PARAMETERS",
                                                   "top_x_brick_count_to_keep");
    config.wall.length =
        ConvertMMtoUM(GetConfigValue<double>(table, "WALL", "length_mm"));
    config.wall.height =
        ConvertMMtoUM(GetConfigValue<double>(table, "WALL", "height_mm"));

    config.fullBrickDimension.length =
        ConvertMMtoUM(GetConfigValue<double>(table, "FULL_BRICK", "length_mm"));
    config.fullBrickDimension.height =
        ConvertMMtoUM(GetConfigValue<double>(table, "FULL_BRICK", "height_mm"));
    config.fullBrickDimension.width =
        ConvertMMtoUM(GetConfigValue<double>(table, "FULL_BRICK", "width_mm"));

    config.halfBrickDimension.length =
        ConvertMMtoUM(GetConfigValue<double>(table, "HALF_BRICK", "length_mm"));
    config.halfBrickDimension.height =
        ConvertMMtoUM(GetConfigValue<double>(table, "HALF_BRICK", "height_mm"));
    config.halfBrickDimension.width =
        ConvertMMtoUM(GetConfigValue<double>(table, "HALF_BRICK", "width_mm"));

    config.jointSize.head =
        ConvertMMtoUM(GetConfigValue<double>(table, "JOINT_SIZE", "head_mm"));
    config.jointSize.bed =
        ConvertMMtoUM(GetConfigValue<double>(table, "JOINT_SIZE", "bed_mm"));

    config.envelope.length =
        ConvertMMtoUM(GetConfigValue<double>(table, "ENVELOPE", "length_mm"));
    config.envelope.height =
        ConvertMMtoUM(GetConfigValue<double>(table, "ENVELOPE", "height_mm"));

  } catch (const std::exception &e) {
    throw std::runtime_error(std::string("Config parsing error: ") + e.what());
  }
  config.ComputeDerivedValues();
  return config;
}

void WallVisualizer::CreateLayout(BondType bond_type) {
  switch (bond_type) {
  case BondType::STRETCHER:
    CreateStretcherLayout();
    break;
  case BondType::ENGLISH_CROSS_BOND:
    CreateEnglishCrossBondLayout();
    break;
  default:
    throw std::runtime_error("Bond type not implemented");
  }
}

size_t WallVisualizer::AddBrickToLayout(std::vector<Brick> &course,
                                        BrickType brickType, size_t row,
                                        size_t column, size_t x_pos,
                                        BrickOrientation orientation) {
  Coordinate coordinate(row, column);
  Position position(x_pos, row * mConfig.courseHeight);

  Brick brick =
      Brick(coordinate, position, brickType, orientation, 0, mTotalBricks);

  mTotalWallArea += brick.GetArea(mConfig);
  course.push_back(brick);
  mTotalBricks++;

  return brick.GetLength(mConfig) + mConfig.jointSize.head;
}

void WallVisualizer::CreateStretcherLayout() {
  mWall.clear();
  mTotalBricks = 0;
  mTotalWallArea = 0;
  auto nr_of_courses = mConfig.wall.height / mConfig.courseHeight;

  for (size_t row = 0; row < nr_of_courses; ++row) {
    std::vector<Brick> course;
    size_t currentWidth = 0;
    size_t column = 0;

    if (row % 2 == 1) {
      currentWidth +=
          AddBrickToLayout(course, BrickType::HALF, row, column, currentWidth,
                           BrickOrientation::STRETCHER);
      column++;
    }

    while (currentWidth + mConfig.fullBrickDimension.length <=
           mConfig.wall.length) {
      currentWidth +=
          AddBrickToLayout(course, BrickType::FULL, row, column, currentWidth,
                           BrickOrientation::STRETCHER);
      column++;
    }

    if (currentWidth + mConfig.halfBrickDimension.length <=
        mConfig.wall.length) {
      currentWidth +=
          AddBrickToLayout(course, BrickType::HALF, row, column, currentWidth,
                           BrickOrientation::STRETCHER);
      column++;
    }

    mWall.push_back(std::move(course));
  }
}

void WallVisualizer::CreateEnglishCrossBondLayout() {
  mWall.clear();
  mTotalBricks = 0;
  mTotalWallArea = 0;
  auto nr_of_courses = mConfig.wall.height / mConfig.courseHeight;

  for (size_t row = 0; row < nr_of_courses; ++row) {
    std::vector<Brick> course;
    size_t current_width = 0;
    size_t column = 0;

    if (row % 2 == 0) {
      if ((row / 2) % 2 == 1) {
        current_width +=
            AddBrickToLayout(course, BrickType::HALF, row, column,
                             current_width, BrickOrientation::STRETCHER);
        column++;
      }

      while (current_width + mConfig.fullBrickDimension.length <=
             mConfig.wall.length) {
        current_width +=
            AddBrickToLayout(course, BrickType::FULL, row, column,
                             current_width, BrickOrientation::STRETCHER);
        column++;
      }

      if (current_width + mConfig.halfBrickDimension.length <=
          mConfig.wall.length) {
        current_width +=
            AddBrickToLayout(course, BrickType::HALF, row, column,
                             current_width, BrickOrientation::STRETCHER);
        column++;
      }

    } else {
      size_t stretcher_width =
          mWall[0].back().position.x + mWall[0].back().GetLength(mConfig);

      current_width +=
          AddBrickToLayout(course, BrickType::HALF, row, column, current_width,
                           BrickOrientation::HEADER);
      column++;

      while (current_width + mConfig.fullBrickDimension.width <=
             stretcher_width) {
        current_width +=
            AddBrickToLayout(course, BrickType::FULL, row, column,
                             current_width, BrickOrientation::HEADER);
        column++;
      }

      if (current_width +
              (static_cast<size_t>(
                  (mConfig.fullBrickDimension.width - mConfig.jointSize.head) *
                  0.5)) <=
          stretcher_width) {
        current_width +=
            AddBrickToLayout(course, BrickType::HALF, row, column,
                             current_width, BrickOrientation::HEADER);
        column++;
      }
    }

    mWall.push_back(std::move(course));
  }
}

bool WallVisualizer::OptimizeBuildOrder() {
  std::cout << "Total robot positions: " << mAllRobotPositions.size()
            << std::endl;

  mBitsetTable.clear();

  std::unordered_map<State, State, StateHash> came_from;

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>>
      priority_queue;

  std::unordered_map<State, double, StateHash> g_scores;
  g_scores.reserve(10 * 1024);

  size_t start_bitset_idx = GetOrCreateBitsetIndex(std::bitset<MAX_BRICKS>());

  auto starting_state = State(0, start_bitset_idx, 0, 0);
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

    if (nodes_explored % 10000 == 0) {
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Explored " << nodes_explored << ", placed "
                << current_node.state.nrOfPlacedBricks << "/" << mTotalBricks
                << ", g(n) " << current_node.cost
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
    if (current_node.state.nrOfPlacedBricks == mTotalBricks) {
      ReconstructPath(came_from, current_node.state);
      std::cout << "\nOptimal solution found!" << std::endl;
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
        Node node;
        node.cost = new_cost;
        node.estimatedTotalCost = new_cost + CalculateHeuristic(state);
        node.state = state;
        node.currentStrideId = current_node.currentStrideId + 1;

        g_scores[node.state] = node.cost;
        came_from[state] = current_node.state;
        priority_queue.push(std::move(node));
      }
    }
  }

  std::cout << "No solution found!" << std::endl;
  return false;
}

std::vector<State> &WallVisualizer::GetPrunedStates(const State &state) {
  mStatesTmp.clear();
  mPrunedStatesTmp.clear();

  for (size_t i = 0; i < mAllRobotPositions.size(); ++i) {
    mStatesTmp.push_back(CreateBrickPlacementState(State(
        i, state.placedBricksIdx, state.nrOfPlacedBricks, state.builtArea)));
  }

  return PruneStates(mStatesTmp, mPrunedStatesTmp);
}

std::vector<State> &
WallVisualizer::PruneStates(const std::vector<State> &states,
                            std::vector<State> &result) {
  mUniqueCountsTmp.clear();
  mTopCountsTmp.clear();

  for (const auto &state : states) {
    if (state.nrOfPlacedBricks > 0) {
      mUniqueCountsTmp.insert(state.nrOfPlacedBricks);
    }
  }

  size_t count = 0;
  for (const auto &value : mUniqueCountsTmp) {
    if (count++ >= mConfig.topXBrickCount)
      break;
    mTopCountsTmp.insert(value);
  }

  for (const auto &state : states) {
    if (mTopCountsTmp.count(state.nrOfPlacedBricks) > 0) {
      result.push_back(state);
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
  auto built_area = state.builtArea;

  bool placed_any = true;
  while (placed_any) {
    placed_any = false;
    for (const auto &brick : reachable_bricks) {

      // Check if already placed
      if (new_placed_bricks.test(brick.index)) {
        continue;
      }

      if (!IsBrickFullySupported(brick, new_placed_bricks)) {
        continue;
      }

      placed_any = true;

      new_placed_bricks.set(brick.index);
      built_area += (brick.brickType == BrickType::FULL)
                        ? mConfig.fullBrickArea
                        : mConfig.halfBrickArea;
      nr_of_placed += 1;
    }
  }

  size_t new_bitset_idx = GetOrCreateBitsetIndex(new_placed_bricks);
  return State(state.robotPositionIdx, new_bitset_idx, nr_of_placed,
               built_area);
}

double WallVisualizer::CalculateHeuristic(const State &state) const {
  auto remaining_area = mTotalWallArea - state.builtArea;

  return static_cast<double>(remaining_area) /
         (static_cast<double>(mConfig.envelope.length) *
          static_cast<double>(mConfig.envelope.height));
}

void WallVisualizer::InitializeRobotPositions() {
  mAllRobotPositions.clear();

  size_t h_step = mConfig.halfBrickDimension.length;
  size_t v_step = mConfig.courseHeight;

  size_t h_start = 0;
  size_t h_end = mConfig.wall.length;
  size_t v_start = 0;
  size_t v_end = mConfig.wall.height;

  for (size_t h_pos = h_start; h_pos < h_end; h_pos += h_step) {
    for (size_t vPos = v_start; vPos < v_end; vPos += v_step) {
      auto pos = Position(h_pos, vPos);
      ;
      mAllRobotPositions.emplace_back(RobotPosition{
          .position = pos,
          .reachableBricks = CalculateReachableBricks(pos),
      });
    }
  }
}
void WallVisualizer::PrecomputeSupportDependencies() {
  for (size_t row_idx = 0; row_idx < mWall.size(); ++row_idx) {
    if (row_idx == 0) {
      continue;
    }

    auto &course = mWall[row_idx];
    for (auto &brick : course) {
      size_t brick_start = brick.position.x;
      size_t brick_end = brick_start + brick.GetLength(mConfig);

      const auto &course_below = mWall[row_idx - 1];
      for (const auto &brick_below : course_below) {
        size_t below_start = brick_below.position.x;
        size_t below_end = below_start + brick_below.GetLength(mConfig) +
                           mConfig.jointSize.head;

        size_t overlap_start = std::max(brick_start, below_start);
        size_t overlap_end = std::min(brick_end, below_end);
        size_t overlap = std::max(static_cast<int64_t>(0),
                                  static_cast<int64_t>(overlap_end) -
                                      static_cast<int64_t>(overlap_start));

        if (overlap > 0) {
          brick.requiredSupportIndices.push_back(brick_below.index);
        }
      }
    }
  }
}

bool WallVisualizer::IsBrickFullySupported(
    const Brick &brick, const std::bitset<MAX_BRICKS> &placed_bricks_bitset) {

  if (brick.requiredSupportIndices.empty()) {
    return true;
  }

  for (auto reqBrickIndex : brick.requiredSupportIndices) {
    if (!placed_bricks_bitset.test(reqBrickIndex)) {
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
                                           const Position &robot_position) {

  size_t brick_start = brick.position.x;
  size_t brick_end = brick_start + brick.GetLength(mConfig);

  size_t brick_bottom_y = brick.position.y;
  size_t brick_top_y = brick_bottom_y + brick.GetHeight(mConfig);

  size_t h_reach_start = robot_position.x;
  size_t h_reach_end = robot_position.x + mConfig.envelope.length;
  size_t v_reach_start = robot_position.y;
  size_t v_reach_end = robot_position.y + mConfig.envelope.height;

  bool h_in_reach = (brick_start >= h_reach_start && brick_end <= h_reach_end);
  bool v_in_reach =
      (brick_bottom_y >= v_reach_start && brick_top_y <= v_reach_end);

  return h_in_reach && v_in_reach;
}

void WallVisualizer::ClearTerminal() {
  std::cout << "\x1b[H\x1b[2J\x1b[3J" << std::flush;
}

void WallVisualizer::Visualize(
    const std::unordered_set<Coordinate, CoordinateHash> &built_bricks) {
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

      if (built_bricks.count(brick.coordinate)) {
        if (brick.orientation == BrickOrientation::HEADER) {
          if (brick.brickType == BrickType::HALF) {
            line += strideColor + "▓|" + color_codes::RESET;
          } else {
            line += strideColor + "▓▓|" + color_codes::RESET;
          }
        } else {
          if (brick.brickType == BrickType::FULL) {
            line += strideColor + "▓▓▓▓▓|" + color_codes::RESET;
          } else {
            line += strideColor + "▓▓▓|" + color_codes::RESET;
          }
        }
      } else {
        if (brick.orientation == BrickOrientation::HEADER) {
          if (brick.brickType == BrickType::HALF) {
            line += color_codes::GRAY + "░|" + color_codes::RESET;
          } else {
            line += color_codes::GRAY + "░░|" + color_codes::RESET;
          }
        } else {
          if (brick.brickType == BrickType::FULL) {
            line += color_codes::GRAY + "░░░░░|" + color_codes::RESET;
          } else {
            line += color_codes::GRAY + "░░░|" + color_codes::RESET;
          }
        }
      }
    }

    printf("Row %2d | %s\n", row, line.c_str());
  }
}

void WallVisualizer::ReconstructPath(
    const std::unordered_map<State, State, StateHash> &came_from,
    State goal_state) {

  mBuildOrder.clear();

  std::vector<State> path;
  State current = goal_state;

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

    for (size_t brick_idx = 0; brick_idx < mTotalBricks; ++brick_idx) {
      if (!prev_bricks.test(brick_idx) && current_bricks.test(brick_idx)) {
        // Find the brick
        for (const auto &course : mWall) {
          for (const auto &brick : course) {
            if (brick.index == brick_idx) {
              Brick built_brick = brick;
              built_brick.strideId = stride_id;
              mBuildOrder.push_back(built_brick);
            }
          }
        }
      }
    }
  }
}

void WallVisualizer::RunInteractiveBuild(BondType bond_type) {
  std::cout << "Visualizing for bond type: " << BondTypeToString(bond_type)
            << std::endl;

  CreateLayout(bond_type);
  PrecomputeSupportDependencies();
  InitializeRobotPositions();

  bool success = OptimizeBuildOrder();
  if (!success) {
    std::cout << "Failed to find valid build order!" << std::endl;
    return;
  }

  std::cout << "Press ENTER to place next brick, 'q' to quit" << std::endl;

  std::unordered_set<Coordinate, CoordinateHash> built_bricks;

  for (size_t idx = 0; idx < mBuildOrder.size(); ++idx) {
    const auto &brick = mBuildOrder[idx];

    std::cout << "\n[" << (idx + 1) << "/" << mBuildOrder.size() << "] "
              << "Place "
              << (brick.brickType == BrickType::FULL ? "FULL" : "HALF")
              << " brick at row " << brick.coordinate.row << ", col "
              << brick.coordinate.column << "? ";

    std::string user_input;
    std::getline(std::cin, user_input);

    user_input.erase(0, user_input.find_first_not_of(" \t\n\r"));
    user_input.erase(user_input.find_last_not_of(" \t\n\r") + 1);
    std::transform(user_input.begin(), user_input.end(), user_input.begin(),
                   ::tolower);

    if (user_input == "q") {
      std::cout << "Build stopped" << std::endl;
      break;
    } else {
      built_bricks.insert(
          Coordinate(brick.coordinate.row, brick.coordinate.column));
      Visualize(built_bricks);
    }
  }

  std::cout << "\nPress ENTER to end program";
  std::string user_input;
  std::getline(std::cin, user_input);
}

} // namespace wall_visualizer