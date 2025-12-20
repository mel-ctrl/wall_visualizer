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

struct SupportInfo {
  int brickIndex;
  int overlap;

  SupportInfo(int idx, int ovr) : brickIndex(idx), overlap(ovr) {}
};

struct BrickSupportDependency {
  std::vector<SupportInfo> supports;
  size_t brickLength;

  BrickSupportDependency() = default;
  BrickSupportDependency(const std::vector<SupportInfo> &s, size_t len)
      : supports(s), brickLength(len) {}
};

struct SupportCacheKey {
  Coordinate coordinate;
  size_t placedBricks;

  bool operator==(const SupportCacheKey &other) const {
    return coordinate == other.coordinate && placedBricks == other.placedBricks;
  }
};

struct SupportCacheKeyHash {
  size_t operator()(const SupportCacheKey &key) const {
    CoordinateHash coordHash;
    size_t h1 = coordHash(key.coordinate);
    size_t h2 = std::hash<size_t>{}(key.placedBricks);
    return h1 ^ (h2 << 1);
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

  Brick() : brickType(BrickType::FULL), strideId(0) {}
  Brick(const Coordinate &coord, const Position &pos, BrickType type,
        int stride)
      : coordinate(coord), position(pos), brickType(type), strideId(stride) {}
};

struct State {
  Position robotPosition;
  std::bitset<MAX_BRICKS> placedBricks;

  State() : placedBricks(0) {}
  State(const Position &pos, std::bitset<MAX_BRICKS> placed)
      : robotPosition(pos), placedBricks(placed) {}

  bool operator==(const State &other) const {
    return robotPosition == other.robotPosition &&
           placedBricks == other.placedBricks;
  }
};

struct StateHash {
  std::size_t operator()(const State &s) const {
    std::size_t h1 = std::hash<int>()(s.robotPosition.x);
    std::size_t h2 = std::hash<int>()(s.robotPosition.y);
    std::size_t h3 = std::hash<std::bitset<MAX_BRICKS>>()(s.placedBricks);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct Node {
  double cost;
  double estimatedTotalCost; // f(n) = g(n) + h(n)
  State state;
  size_t currentStrideId;
  std::vector<Brick> brickHistory;
  uint64_t remainingArea;

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
  Config mConfig;
  std::vector<std::vector<Brick>> mWall;
  std::vector<Brick> mBuildOrder;

  uint64_t mTotalBricks = 0;
  double mTotalCost = 0;
  uint64_t mTotalStrides = 0;
  uint64_t mTotalWallArea = 0;

  // Caches
  std::unordered_map<Position, std::unordered_set<Coordinate, CoordinateHash>,
                     PositionHash>
      mReachabilityCache;
  std::unordered_map<Coordinate, int, CoordinateHash> mBrickToIndex;

  std::unordered_map<Coordinate, BrickSupportDependency, CoordinateHash>
      mSupportDependencies;

  std::unordered_map<SupportCacheKey, bool, SupportCacheKeyHash>
      mSupportCheckCache;

  Config LoadConfig(const std::string &fileName);
  size_t ConvertMMtoUM(double value);

  void CreateLayout(BondType bondType);
  void CreateStretcherLayout();

  bool OptimizeBuildOrder();

  std::vector<Position> GetAllPossibleRobotPositions();
  void PrecomputeSupportDependencies();
  void CalculateReachableBricks(const Position &robotPosition);

  bool IsBrickFullySupported(const Coordinate &coordinate,
                             const std::bitset<MAX_BRICKS> &placedBricksBitset);
  bool CalculateBrickInReach(const Brick &brick, const Position &robotPosition);

  std::vector<Node> GenerateBrickPlacementNeighbors(const Node &currentNode);
  std::vector<Node>
  GenerateMovementNeighbors(const Node &currentNode,
                            const std::vector<Position> &allPositions);

  double CalculateHeuristic(uint64_t remainingArea) const;

  void
  Visualize(const std::unordered_set<Coordinate, CoordinateHash> &builtBricks);
  void ClearTerminal();

  int CountSetBits(size_t n) const { return __builtin_popcountll(n); }
};

} // namespace wall_visualizer