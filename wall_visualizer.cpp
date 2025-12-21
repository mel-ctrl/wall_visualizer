#include "wall_visualizer.hpp"
#include "lib/toml.hpp"
#include <algorithm>
#include <bitset>
#include <cstdint>
#include <iostream>
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
  const std::vector<std::string> colors = {RED,     GREEN, YELLOW, BLUE,
                                           MAGENTA, CYAN,  ORANGE, PURPLE,
                                           PINK,    GRAY,  RESET};
  return colors[stride_id % colors.size()];
}
} // namespace color_codes

WallVisualizer::WallVisualizer(const std::string &config_path)
    : mTotalBricks(0), mTotalCost(0), mTotalStrides(0), mTotalWallArea(0) {
  mConfig = LoadConfig(config_path);
  mNeighborsTmp.reserve(64 * 1024);
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
  switch (bondType) {
  case BondType::STRETCHER:
    CreateStretcherLayout();
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
                         auto row, auto column, auto x_pos) -> int {
    Coordinate coordinate(row, column);
    Position position(x_pos, row * mConfig.courseHeight);

    course.push_back(Brick(coordinate, position, brickType, 0));

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
      currentWidth +=
          addBrick(course, BrickType::HALF, row, column, currentWidth);
      column++;
    }

    while (currentWidth + mConfig.fullBrickDimension.length <=
           mConfig.wall.length) {
      currentWidth +=
          addBrick(course, BrickType::FULL, row, column, currentWidth);
      column++;
    }

    if (currentWidth + mConfig.halfBrickDimension.length <=
        mConfig.wall.length) {
      currentWidth +=
          addBrick(course, BrickType::HALF, row, column, currentWidth);
      column++;
    }

    mTotalBricks += course.size();
    mWall.push_back(std::move(course));
  }

  int64_t index = 0;
  for (const auto &course : mWall) {
    for (const auto &brick : course) {
      mBrickToIndex[brick.coordinate] = index++;
    }
  }

  // Debug output
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
                << std::endl;
    }
  }
}

bool WallVisualizer::OptimizeBuildOrder() {
  auto allPositions = GetAllPossibleRobotPositions();

  // Precompute caches
  for (const auto &position : allPositions) {
    CalculateReachableBricks(position);
  }

  PrecomputeSupportDependencies();

  std::cout << "Total robot positions: " << allPositions.size() << std::endl;
  std::cout << "Total bricks: " << mTotalBricks << std::endl;
  std::cout << "Total wall area: " << mTotalWallArea << std::endl;

  // DEBUG: Check reachability
  std::cout << "\n=== REACHABILITY DEBUG ===" << std::endl;
  std::unordered_set<Coordinate, CoordinateHash> allReachableBricks;
  for (const auto &pos : allPositions) {
    const auto &reachable = mReachabilityCache[pos];
    for (const auto &coord : reachable) {
      allReachableBricks.insert(coord);
    }
  }
  std::cout << "Total bricks reachable from ANY position: "
            << allReachableBricks.size() << " / " << mTotalBricks << std::endl;

  if (allReachableBricks.size() < mTotalBricks) {
    std::cout << "ERROR: Some bricks are NEVER reachable!" << std::endl;
    // Find which bricks are unreachable
    for (const auto &course : mWall) {
      for (const auto &brick : course) {
        if (allReachableBricks.find(brick.coordinate) ==
            allReachableBricks.end()) {
          std::cout << "  Unreachable: Row " << brick.coordinate.row << ", Col "
                    << brick.coordinate.column << " at x=[" << brick.position.x
                    << ", "
                    << (brick.position.x +
                        (brick.brickType == BrickType::FULL
                             ? mConfig.fullBrickDimension.length
                             : mConfig.halfBrickDimension.length))
                    << "]"
                    << ", y=[" << brick.position.y << ", "
                    << (brick.position.y +
                        (brick.brickType == BrickType::FULL
                             ? mConfig.fullBrickDimension.height
                             : mConfig.halfBrickDimension.height))
                    << "]" << std::endl;
        }
      }
    }
  }

  // DEBUG: Check support dependencies
  std::cout << "\n=== SUPPORT DEPENDENCIES DEBUG ===" << std::endl;
  for (size_t row = 0; row < std::min(size_t(3), mWall.size()); ++row) {
    for (size_t col = 0; col < mWall[row].size(); ++col) {
      const auto &brick = mWall[row][col];
      auto depIt = mSupportDependencies.find(brick.coordinate);

      if (row == 0) {
        std::cout << "Row " << row << ", Col " << col
                  << ": Ground level (no support needed)" << std::endl;
      } else if (depIt != mSupportDependencies.end()) {
        const auto &dep = depIt->second;
        std::cout << "Row " << row << ", Col " << col << ": needs "
                  << dep.brickLength << " support, has " << dep.supports.size()
                  << " supporting bricks:" << std::endl;
        for (const auto &sup : dep.supports) {
          std::cout << "    Brick index " << sup.brickIndex << " provides "
                    << sup.overlap << " overlap" << std::endl;
        }
      } else {
        std::cout << "Row " << row << ", Col " << col
                  << ": ERROR - no support dependency found!" << std::endl;
      }
    }
  }

  State startingState(Position(0, 0), std::bitset<MAX_BRICKS>(), 0);

  std::unordered_map<State, double, StateHash> gScores;
  gScores[startingState] = 0.0;
  gScores.reserve(1000000);

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>>
      priorityQueue;

  Node startNode;
  startNode.cost = 0.0;
  startNode.estimatedTotalCost = CalculateHeuristic(mTotalWallArea);
  startNode.state = startingState;
  startNode.currentStrideId = 0;
  startNode.remainingArea = mTotalWallArea;

  priorityQueue.push(std::move(startNode));

  size_t nodesExplored = 0;

  while (!priorityQueue.empty()) {
    Node currentNode = priorityQueue.top();
    priorityQueue.pop();

    nodesExplored++;

    if (nodesExplored % 1000 == 0) {
      size_t nrPlaced = currentNode.state.nrOfPlacedBricks;
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Explored " << nodesExplored << ", placed " << nrPlaced
                << "/" << mTotalBricks << ", cost " << currentNode.cost
                << ", f(n)=" << currentNode.estimatedTotalCost
                << ", h(n)=" << CalculateHeuristic(currentNode.remainingArea)
                << ", queue " << priorityQueue.size() << std::endl;
    }

    // Check if we've found a better path to this state
    auto it = gScores.find(currentNode.state);
    if (it != gScores.end() && currentNode.cost > it->second) {
      continue;
    }

    // Goal check
    size_t nrOfBricksPlaced = currentNode.state.nrOfPlacedBricks;
    if (nrOfBricksPlaced == mTotalBricks) {
      mTotalCost = currentNode.cost;
      mTotalStrides = currentNode.currentStrideId;
      std::cout << "\n✓ Optimal solution found!" << std::endl;
      std::cout << "  Total cost: " << mTotalCost << std::endl;
      std::cout << "  Total strides: " << mTotalStrides << std::endl;
      std::cout << "  Nodes explored: " << nodesExplored << std::endl;
      return true;
    }

    mNeighborsTmp.clear();
    // Generate neighbors
    GenerateBrickPlacementNeighbors(mNeighborsTmp, currentNode);
    GenerateMovementNeighbors(mNeighborsTmp, currentNode, allPositions);

    // Process all neighbors
    for (auto &neighbor : mNeighborsTmp) {
      auto it = gScores.find(neighbor.state);
      if (it == gScores.end() || neighbor.cost < it->second) {
        gScores[neighbor.state] = neighbor.cost;
        priorityQueue.push(std::move(neighbor));
      }
    }
  }

  std::cout << "✗ No solution found!" << std::endl;
  return false;
}

void WallVisualizer::GenerateBrickPlacementNeighbors(std::vector<Node> &result,
                                                     const Node &currentNode) {

  const auto &reachableCoords =
      mReachabilityCache[currentNode.state.robotPosition];

  for (const auto &coordinate : reachableCoords) {
    int brickIndex = mBrickToIndex[coordinate];

    // Check if already placed
    if (currentNode.state.placedBricks.test(brickIndex)) {
      continue;
    }

    // Check support
    if (!IsBrickFullySupported(coordinate, currentNode.state.placedBricks)) {
      continue;
    }

    std::bitset<MAX_BRICKS> newPlaced = currentNode.state.placedBricks;
    newPlaced.set(brickIndex);
    State newState(currentNode.state.robotPosition, newPlaced,
                   currentNode.state.nrOfPlacedBricks + 1);

    const Brick &designBrick = mWall[coordinate.row][coordinate.column];
    int64_t brickArea = (designBrick.brickType == BrickType::FULL)
                            ? mConfig.fullBrickArea
                            : mConfig.halfBrickArea;
    int64_t newRemainingArea =
        std::max(uint64_t(0), currentNode.remainingArea - brickArea);

    Node neighbor;
    neighbor.cost = currentNode.cost;
    neighbor.estimatedTotalCost =
        neighbor.cost + CalculateHeuristic(newRemainingArea);
    neighbor.state = newState;
    neighbor.currentStrideId = currentNode.currentStrideId;
    neighbor.remainingArea = newRemainingArea;

    result.push_back(std::move(neighbor));
  }
}

void WallVisualizer::GenerateMovementNeighbors(
    std::vector<Node> &result, const Node &currentNode,
    const std::vector<Position> &allPositions) {

  for (const auto &newPos : allPositions) {
    if (newPos == currentNode.state.robotPosition) {
      continue;
    }
    /*
    const auto &reachableCoords =
        mReachabilityCache[currentNode.state.robotPosition];

    bool could_place_any_brick = false;
    for (const auto &coordinate : reachableCoords) {
      int brickIndex = mBrickToIndex[coordinate];

      if (!currentNode.state.placedBricks.test(brickIndex) &&
          IsBrickFullySupported(coordinate, currentNode.state.placedBricks)) {
        could_place_any_brick = true;
        break;
      }
    }

    if (!could_place_any_brick) {
      continue;
    }
    */

    State newState(newPos, currentNode.state.placedBricks,
                   currentNode.state.nrOfPlacedBricks);

    double newCost = currentNode.cost + 1.0;
    double newEstimatedTotalCost =
        newCost + CalculateHeuristic(currentNode.remainingArea);

    Node neighbor;
    neighbor.cost = newCost;
    neighbor.estimatedTotalCost = newEstimatedTotalCost;
    neighbor.state = newState;
    neighbor.currentStrideId = currentNode.currentStrideId + 1;
    neighbor.remainingArea = currentNode.remainingArea;

    result.push_back(std::move(neighbor));
  }
}

double WallVisualizer::CalculateHeuristic(uint64_t remainingArea) const {
  return static_cast<double>(remainingArea) /
         static_cast<double>(
             (mConfig.envelope.length * mConfig.envelope.height));
}

std::vector<Position> WallVisualizer::GetAllPossibleRobotPositions() {
  std::vector<Position> positions;

  int hStep = mConfig.halfBrickDimension.length;
  int vStep = mConfig.halfBrickDimension.height;

  int hStart = 0;
  int hEnd = mConfig.wall.length;
  int vStart = 0;
  int vEnd = mConfig.wall.height;

  std::cout << "DEBUG Position generation:" << std::endl;
  std::cout << "  H: " << hStart << " to " << hEnd << " step " << hStep
            << std::endl;
  std::cout << "  V: " << vStart << " to " << vEnd << " step " << vStep
            << std::endl;

  for (int hPos = hStart; hPos < hEnd; hPos += hStep) {
    for (int vPos = vStart; vPos < vEnd; vPos += vStep) {
      positions.emplace_back(hPos, vPos);
    }
  }

  std::cout << "  Total positions: " << positions.size() << std::endl;

  return positions;
}

void WallVisualizer::PrecomputeSupportDependencies() {
  for (size_t rowIdx = 0; rowIdx < mWall.size(); ++rowIdx) {
    if (rowIdx == 0)
      continue;

    const auto &course = mWall[rowIdx];
    for (const auto &brick : course) {
      int brickStart = brick.position.x;
      int brickLen = (brick.brickType == BrickType::FULL)
                         ? mConfig.fullBrickDimension.length
                         : mConfig.halfBrickDimension.length;
      int brickEnd = brickStart + brickLen;

      std::vector<SupportInfo> supports;
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
          supports.emplace_back(mBrickToIndex[brickBelow.coordinate], overlap);
        }
      }

      mSupportDependencies[brick.coordinate] =
          BrickSupportDependency(supports, brickLen);
    }
  }
}

bool WallVisualizer::IsBrickFullySupported(
    const Coordinate &coordinate,
    const std::bitset<MAX_BRICKS> &placedBricksBitset) {
  if (coordinate.row == 0) {
    return true;
  }

  auto depIt = mSupportDependencies.find(coordinate);
  if (depIt == mSupportDependencies.end()) {
    return false;
  }

  const BrickSupportDependency &dependency = depIt->second;
  size_t supportedLength = 0;

  for (const auto &supportInfo : dependency.supports) {
    if (placedBricksBitset.test(supportInfo.brickIndex)) {
      supportedLength += supportInfo.overlap;

      if (supportedLength >= dependency.brickLength) {
        return true;
      }
    }
  }

  return false;
}

void WallVisualizer::CalculateReachableBricks(const Position &robotPosition) {
  auto it = mReachabilityCache.find(robotPosition);
  if (it != mReachabilityCache.end()) {
    return;
  }

  std::unordered_set<Coordinate, CoordinateHash> reachable;

  for (const auto &course : mWall) {
    for (const auto &brick : course) {
      if (CalculateBrickInReach(brick, robotPosition)) {
        reachable.insert(brick.coordinate);
      }
    }
  }

  mReachabilityCache[robotPosition] = std::move(reachable);
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
          line += strideColor + "▓▓▓▓" + color_codes::RESET + " ";
        } else {
          line += strideColor + "▓▓" + color_codes::RESET + " ";
        }
      } else {
        if (brick.brickType == BrickType::FULL) {
          line += color_codes::GRAY + "░░░░" + color_codes::RESET + "  ";
        } else {
          line += color_codes::GRAY + "░░" + color_codes::RESET + " ";
        }
      }
    }

    printf("Row %2d | %s\n", row, line.c_str());
  }
}

void WallVisualizer::RunInteractiveBuild(BondType bond_type) {
  std::cout << "Visualizing for bond type STRETCHER" << std::endl;

  CreateLayout(bond_type);

  bool success = OptimizeBuildOrder();
  if (!success) {
    std::cout << "Failed to find valid build order!" << std::endl;
    return;
  }
  /*
  std::cout << "\nOptimal strategy found: total cost = " << mTotalCost
            << ", strides = " << mTotalStrides << std::endl;
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
  */
}

} // namespace wall_visualizer