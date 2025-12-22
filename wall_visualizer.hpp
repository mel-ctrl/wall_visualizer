#pragma once

#include <bitset>
#include <cstdint>
#include <functional>
#include <unordered_set>

namespace wall_visualizer {

constexpr size_t MAX_BRICKS = 512;

enum class BrickType { FULL, HALF };

enum class BondType { STRETCHER, ENGLISH_CROSS_BOND, WILD_BOND };

struct Position {
  int64_t x = 0;
  int64_t y = 0;

  Position() : x(0), y(0) {}
  Position(int x_, int y_) : x(x_), y(y_) {}

  bool operator==(const Position &other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const Position &other) const { return !(*this == other); }
};

struct Coordinate {
  size_t row;
  size_t column;

  Coordinate() : row(0), column(0) {}
  Coordinate(int row_, int column_) : row(row_), column(column_) {}

  bool operator==(const Coordinate &other) const {
    return row == other.row && column == other.column;
  }

  bool operator!=(const Coordinate &other) const { return !(*this == other); }
};

struct PositionHash {
  std::size_t operator()(const Position &p) const {
    return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
  }
};

struct CoordinateHash {
  std::size_t operator()(const Coordinate &c) const {
    return std::hash<int>()(c.row) ^ (std::hash<int>()(c.column) << 1);
  }
};

struct BrickDimension {
  size_t length = 0;
  size_t height = 0;
  size_t width = 0;
};

struct JointSize {
  size_t head = 0;
  size_t bed = 0;
};

struct RobotEnvelope {
  size_t length = 0;
  size_t height = 0;
};

struct Wall {
  size_t length = 0;
  size_t height = 0;
};

struct Config {
  Wall wall;
  BrickDimension fullBrickDimension;
  BrickDimension halfBrickDimension;
  JointSize jointSize;
  RobotEnvelope envelope;

  size_t courseHeight = 0;
  size_t fullBrickWithJointLength = 0;
  size_t halfBrickWithJointLength = 0;
  size_t fullBrickArea = 0;
  size_t halfBrickArea = 0;

  void ComputeDerivedValues() {
    courseHeight = fullBrickDimension.height + jointSize.bed;
    fullBrickWithJointLength = fullBrickDimension.length + jointSize.head;
    halfBrickWithJointLength = halfBrickDimension.length + jointSize.head;
    fullBrickArea = fullBrickDimension.length * fullBrickDimension.height;
    halfBrickArea = halfBrickDimension.length * halfBrickDimension.height;
  }
};

struct Brick {
  Coordinate coordinate;
  Position position;
  BrickType brickType = BrickType::FULL;
  size_t strideId = 0;
  size_t index = 0;
  std::vector<size_t> requiredSupportIndices; // these are indices too

  Brick() : brickType(BrickType::FULL), strideId(0), index(0) {}
  Brick(const Coordinate &coord, const Position &pos, BrickType type,
        int stride, size_t idx)
      : coordinate(coord), position(pos), brickType(type), strideId(stride),
        index(idx) {}
};

struct RobotPosition {
  Position position;
  std::vector<Brick> reachableBricks;
};

struct State {
  size_t robotPositionIdx = 0;
  size_t placedBricksIdx = 0; // Index into mBitsetTable
  uint64_t nrOfPlacedBricks = 0;

  State() : robotPositionIdx(0), placedBricksIdx(0), nrOfPlacedBricks(0) {}
  State(int posIdx, size_t bricksIdx, size_t nrPlaced)
      : robotPositionIdx(posIdx), placedBricksIdx(bricksIdx),
        nrOfPlacedBricks(nrPlaced) {}

  bool operator==(const State &other) const {
    return robotPositionIdx == other.robotPositionIdx &&
           placedBricksIdx == other.placedBricksIdx;
  }
};

struct StateHash {
  std::size_t operator()(const State &state) const {
    return state.robotPositionIdx + (state.placedBricksIdx << 16);
  }
};

struct Node {
  double cost = 0;
  double estimatedTotalCost = 0; // f(n) = g(n) + h(n)
  State state;
  size_t currentStrideId = 0;
  uint64_t remainingArea = 0;

  // For priority queue (min-heap based on estimatedTotalCost)
  bool operator>(const Node &other) const {
    return estimatedTotalCost > other.estimatedTotalCost;
  }
};

class WallVisualizer {
public:
  explicit WallVisualizer(const std::string &configFile);

  void RunInteractiveBuild(BondType bond_type);

private:
  std::vector<Node> mNeighborsTmp;
  Node mNodeTmp;
  Config mConfig;
  std::vector<std::vector<Brick>> mWall;
  std::vector<Brick> mBuildOrder;

  std::vector<size_t> mRobotPosIdxWithMaxPlaceableBricksTmp;

  std::vector<RobotPosition> mAllRobotPositions;

  std::vector<std::bitset<MAX_BRICKS>> mBitsetTable;
  std::unordered_map<std::bitset<MAX_BRICKS>, size_t> mBitsetToIndex;

  uint64_t mTotalBricks = 0;
  uint64_t mTotalWallArea = 0;

  // Caches
  std::unordered_map<Position, std::unordered_set<Coordinate, CoordinateHash>,
                     PositionHash>
      mReachabilityCache;

  Config LoadConfig(const std::string &fileName);
  size_t ConvertMMtoUM(double value);

  void CreateLayout(BondType bondType);
  void CreateStretcherLayout();

  bool OptimizeBuildOrder();

  void InitializeRobotPositions();
  void PrecomputeSupportDependencies();
  std::vector<Brick> CalculateReachableBricks(const Position &position);

  bool IsBrickFullySupported(const Brick &brick,
                             const std::bitset<MAX_BRICKS> &placedBricksBitset);
  bool CalculateBrickInReach(const Brick &brick, const Position &robotPosition);

  void GenerateMovementNeighbors(auto &g_scores, auto &came_from, auto &queue,
                                 const Node &current_node);

  bool GenerateBrickPlacementNeighbors(auto &g_scores, auto &came_from,
                                       auto &queue, const Node &current_node);

  double CalculateHeuristic(uint64_t remainingArea) const;
  void
  ReconstructPath(const std::unordered_map<State, State, StateHash> &came_from,
                  State goalState);
  void
  Visualize(const std::unordered_set<Coordinate, CoordinateHash> &builtBricks);
  void ClearTerminal();

  int CountSetBits(size_t n) const { return __builtin_popcountll(n); }

  inline size_t GetOrCreateBitsetIndex(const std::bitset<MAX_BRICKS> &bitset) {
    auto it = mBitsetToIndex.find(bitset);
    if (it != mBitsetToIndex.end()) {
      return it->second;
    }

    size_t newIndex = mBitsetTable.size();
    mBitsetTable.push_back(bitset);
    mBitsetToIndex[bitset] = newIndex;
    return newIndex;
  }
};

} // namespace wall_visualizer