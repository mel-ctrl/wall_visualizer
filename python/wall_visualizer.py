import heapq
import tomllib
import argparse
import wall_visualizer_types as types
from typing import Dict, List, Set
from dataclasses import dataclass, field
import cProfile
import pstats
from pstats import SortKey

#TODO: A* takes a very long time to find a globally optimum solution due to too many states. 
# Total cost f(n) = g(n) + h(n): It tries to greedily reduce total cost g(n) (so building as many bricks from current position) + a lower bound for the estimation of total cost remaining so it prioritizes 
# visiting states that are not increasing the total cost
# A very low bound for the heuristic would be -> area to build / envelope area. 
# This is constant for each state with an added brick in the middle of the wall (does not matter if you place a brick at positionA or positionB, heuristic is the same if the added brick
# is same size which is the case for the middle part of a wall with a brick pattern of same sized bricks).
# Since I cannot manage to reduce the states down and still guarantee global optimum soltion. I will consider simplifying it by greedily adding as many bricks as possible in the same position and 
# then generate a new state with the new wall instead of a new state per added brick. So this means that we still try to find the robot path that reduces amount of movements
# but maybe less optimal (especially for very random wall patterns) since we greedily add all reachable and supported bricks from same position first.

@dataclass(frozen=True)
class State:
    robotPosition: types.Position
    placedBricks: int = 0

    _hash_cache: int = field(default=None, init=False, compare=False, hash=False)

    def __hash__(self):
        if object.__getattribute__(self, '_hash_cache') is None:
            h = hash((self.robotPosition.x, self.robotPosition.y, self.placedBricks))
            object.__setattr__(self, '_hash_cache', h)
            return h
        return object.__getattribute__(self, '_hash_cache')
    
    def __eq__(self, other):
        return (self.robotPosition.x == other.robotPosition.x and 
                self.robotPosition.y == other.robotPosition.y and 
                self.placedBricks == other.placedBricks)

    
@dataclass
class Node:
    cost: float
    estimatedTotalCost: float # this is for A* f(n) = g(n) + h(n)
    state: State
    currentStrideId: int
    brickHistory: List[types.Brick]
    remainingArea: int

    def __lt__(self, other):
        return self.estimatedTotalCost < other.estimatedTotalCost

class WallVisualizer:
    def __init__(self, config_file: str):
        self.config = self.LoadConfig(config_file)
        self.wall: List[List[types.Brick]] = []
        self.buildOrder: List[types.Brick] = []
        
        self.totalBricks: int = 0
        self.totalCost: int = 0
        self.totalStrides: int = 0 

        self.reachabilityCache = {}
        self.brickToIndex: Dict[types.Coordinate, int] = {}
        self.totalWallArea: int = 0

        self.supportDependencies = {}
        self.supportCheckCache = {}

    def LoadConfig(self, file_name) -> types.Config:
        with open(file_name, 'rb') as f:
            data = tomllib.load(f)

        return types.Config(
            wall = types.Wall(
                length = self._ConvertMMtoUM(data["WALL"]["length_mm"]),
                height = self._ConvertMMtoUM(data["WALL"]["height_mm"]),
                ),
            
            fullBrickDimension = types.BrickDimension(
                length = self._ConvertMMtoUM(data["FULL_BRICK"]["length_mm"]), 
                height = self._ConvertMMtoUM(data["FULL_BRICK"]["height_mm"]),
                width = self._ConvertMMtoUM(data["FULL_BRICK"]["width_mm"]),
                ),
            
            halfBrickDimension = types.BrickDimension(
                length = self._ConvertMMtoUM(data["HALF_BRICK"]["length_mm"]), 
                height = self._ConvertMMtoUM(data["HALF_BRICK"]["height_mm"]),
                width = self._ConvertMMtoUM(data["HALF_BRICK"]["width_mm"]),
                ),
            
            jointSize = types.JointSize(
                head = self._ConvertMMtoUM(data["JOINT_SIZE"]["head_mm"]), 
                bed = self._ConvertMMtoUM(data["JOINT_SIZE"]["bed_mm"]),
                ),

            envelope = types.RobotEnvelope(
                length = self._ConvertMMtoUM(data["ENVELOPE"]["length_mm"]), 
                height = self._ConvertMMtoUM(data["ENVELOPE"]["height_mm"]),
                ),
            )
    def _ConvertMMtoUM(self, value: float):
        return int(round(value * 1000))


    def _ClearTerminal(self):
        print("\x1b[H\x1b[2J\x1b[3J", end="", flush=True)
        
    def _Visualize(self, builtBricks: Set[types.Coordinate])-> None:
        self._ClearTerminal()

        brick_stride_map = {}
        for brick in self.buildOrder:
            brick_stride_map[brick.coordinate] = brick.strideId

        for row in reversed(range(len(self.wall))):
            course = self.wall[row]
            line = ""
            
            for brick in course:
                brick_key = brick.coordinate
                stride_id = brick_stride_map.get(brick_key)
                strideColor = types.Color.GetStrideColor(stride_id)
                if brick_key in builtBricks:
                    if brick.brickType == types.BrickType.FULL:
                        line += f"{strideColor}▓▓▓▓{types.Color.RESET} "
                    else:
                        line += f"{strideColor}▓▓{types.Color.RESET} "
                else:
                    if brick.brickType == types.BrickType.FULL:
                        line += f"{types.Color.GRAY}░░░░{types.Color.RESET}  "
                    else:
                        line += f"{types.Color.GRAY}░░{types.Color.RESET} "

            print(f"Row {row:2d} | {line}")
            
    def RunInteractiveBuild(self, bond_type: types.BondType) -> None:
        print("Visualizing for bond type \"{}\"".format(bond_type))
        self._CreateLayout(bond_type)
        success = self._OptimizeBuildOrder()
        if not success:
            print("Failed to find valid build order!")
            return
        '''
        print(f"Optimal strategy found: total cost = {self.totalCost}, strides = {self.totalStrides}")
        print("Press ENTER to place next brick, 'q' to quit")

        builtBricks: Set[types.Coordinate] = set()
        for idx, brick in enumerate(self.buildOrder):
            user_input = input(f"\n[{idx + 1}/{len(self.buildOrder)}] "
                               f"Place {brick.brickType.value} brick at row {brick.coordinate.row}, col {brick.coordinate.column}? ").strip().lower()
            if user_input == 'q':
                print("Build stopped")
                break
            else:
                builtBricks.add(types.Coordinate(brick.coordinate.row, brick.coordinate.column))
                self._Visualize(builtBricks)
        
        user_input = input(f"\nPress ENTER to end program")
        '''

    def _CreateLayout(self, bond_type: types.BondType) -> None:
        match(bond_type):
            case types.BondType.STRETCHER:
                return self._CreateStretcherLayout()
            #case types.BondType.ENGLISH_CROSS_BOND:
            #    return self._CreateEnglishCrossLayout()
            #case types.BondType.WILD_BOND:
            #    return self._CreateWildBondLayout()
            case _:
                raise ValueError(f"bond type \"{bond_type}\" not implemented") 

    def _OptimizeBuildOrder(self) -> bool:

        all_positions = self._GetAllPossibleRobotPositions()
        # Create cache of reachable bricks from each robot coordinate
        for position in all_positions:
            self._CalculateReachableBricks(position)
        # Create cache of support dependencies for each brick in design wall
        self._PrecomputeSupportDependencies()
        
        print(f"Total robot positions: {len(all_positions)}")
        print(f"Total bricks: {self.totalBricks}")
        print(f"Total wall area: {self.totalWallArea}")
        

        starting_state = State(robotPosition=types.Position(0, 0), placedBricks=0)

        gScores: Dict = {starting_state: 0}

        priority_queue = []
        heapq.heappush(priority_queue, Node(cost=0,
                                            estimatedTotalCost=0+self._CalculateHeuristic(self.totalWallArea),
                                            state=starting_state, 
                                            currentStrideId=0, 
                                            brickHistory=[], 
                                            remainingArea=self.totalWallArea))

        nodes_explored = 0

        while priority_queue:
            current_node = heapq.heappop(priority_queue)
            nodes_explored += 1

            if nodes_explored % 1000 == 0:
                nr_placed = bin(current_node.state.placedBricks).count('1')
                print(f"Explored {nodes_explored}, placed {nr_placed}/{self.totalBricks}, "
                    f"cost {current_node.cost}, f(n)={current_node.estimatedTotalCost}, "
                    f"queue {len(priority_queue)}")
            

            if current_node.state in gScores and current_node.cost > gScores[current_node.state]:
                continue
            
            nr_of_bricks_placed = bin(current_node.state.placedBricks).count('1')
            if nr_of_bricks_placed == self.totalBricks:
                self.buildOrder = current_node.brickHistory
                self.totalCost = current_node.cost
                self.totalStrides = current_node.currentStrideId
                return True

            neighbors = list(self._GenerateBrickPlacementNeighbors(current_node)) + list(self._GenerateMovementNeighbors(current_node, all_positions))

            for neighbor in neighbors:
                 if neighbor.state not in gScores or neighbor.cost < gScores[neighbor.state]:
                    gScores[neighbor.state] = neighbor.cost
                    heapq.heappush(priority_queue, neighbor)

        return False
    
    def _GenerateBrickPlacementNeighbors(self, current_node: Node):
        pos_key = (current_node.state.robotPosition.x, current_node.state.robotPosition.y)
        reachable_coords = self.reachabilityCache[pos_key]
        
        for coordinate in reachable_coords:
            brick_index = self.brickToIndex[coordinate]
            
            # Check if already placed
            if (current_node.state.placedBricks >> brick_index) & 1:
                continue
            
            if not self._IsBrickFullySupported(coordinate, current_node.state.placedBricks):
                continue

            new_placed = current_node.state.placedBricks | (1 << brick_index)
        
            new_state = State(
            robotPosition=current_node.state.robotPosition,
            placedBricks=new_placed
            )

            design_brick = self.wall[coordinate.row][coordinate.column]
            new_remaining_area = max(0, current_node.remainingArea - (self.config.fullBrickArea if design_brick.brickType == types.BrickType.FULL else self.config.halfBrickArea))
            
            new_cost = current_node.cost
            newEstimatedTotalCost = new_cost + self._CalculateHeuristic(new_remaining_area)

            yield Node(
                cost=new_cost,
                estimatedTotalCost=newEstimatedTotalCost,
                state=new_state,
                currentStrideId=current_node.currentStrideId,
                brickHistory=current_node.brickHistory + [
                    types.Brick(coordinate, design_brick.position, design_brick.brickType, current_node.currentStrideId)
                ],
                remainingArea=new_remaining_area
            )

    def _GenerateMovementNeighbors(self, current_node: Node, all_positions: List[types.Position]):
        for new_pos in all_positions:
            if new_pos == current_node.state.robotPosition:
                continue

            pos_key = (new_pos.x, new_pos.y)
            reachable_from_new_pos = self.reachabilityCache[pos_key]
            
            could_place_any_brick = False
            for coordinate in reachable_from_new_pos:
                brick_index = self.brickToIndex[coordinate]
                if not ((current_node.state.placedBricks >> brick_index) & 1):
                    if self._IsBrickFullySupported(coordinate, current_node.state.placedBricks):
                        could_place_any_brick = True
                        break
            
            if not could_place_any_brick:
                continue

            new_state = State(
                robotPosition=new_pos,
                placedBricks=current_node.state.placedBricks,
            )

            new_cost = current_node.cost + 1.0
            newEstimatedTotalCost = new_cost + self._CalculateHeuristic(current_node.remainingArea)

            yield Node(
                cost=new_cost,
                estimatedTotalCost=newEstimatedTotalCost,
                state=new_state,
                currentStrideId=current_node.currentStrideId + 1,
                brickHistory=current_node.brickHistory,
                remainingArea=current_node.remainingArea
            )

    def _CalculateHeuristic(self, remaining_area) -> float:
        return remaining_area / (self.config.envelope.length * self.config.envelope.height)
    
    def _GetAllPossibleRobotPositions(self) -> List[types.Position]:
        positions = []
            
        h_step = int(self.config.halfBrickDimension.length)
        v_step = int(self.config.halfBrickDimension.height)
        
        h_start = 0
        h_end = int(self.config.wall.length)
        
        v_start = 0
        v_end = int(self.config.wall.height)
        
        print(f"DEBUG Position generation:")
        print(f"  H: {h_start} to {h_end} step {h_step}")
        print(f"  V: {v_start} to {v_end} step {v_step}")
        
        for h_pos in range(h_start, h_end, h_step):
            for v_pos in range(v_start, v_end, v_step):
                positions.append(types.Position(h_pos, v_pos))
        
        print(f"  Sample positions: {positions[:10]}")
        print(f"  Unique Y values: {sorted(set(p.y for p in positions))}")
        
        return positions


    def _PrecomputeSupportDependencies(self):
        for row_idx, course in enumerate(self.wall):
            if row_idx == 0:
                continue
            
            for brick in course:
                brick_start = brick.position.x
                brick_len = (self.config.fullBrickDimension.length 
                            if brick.brickType == types.BrickType.FULL 
                            else self.config.halfBrickDimension.length)
                brick_end = brick_start + brick_len
                
                supports = []
                course_below = self.wall[row_idx - 1]
                
                for brick_below in course_below:
                    below_start = brick_below.position.x
                    below_len = (self.config.fullBrickDimension.length 
                                if brick_below.brickType == types.BrickType.FULL
                                else self.config.halfBrickDimension.length)
                    below_end = below_start + (below_len + self.config.jointSize.head)
                    
                    overlap_start = max(brick_start, below_start)
                    overlap_end = min(brick_end, below_end)
                    overlap = max(0, overlap_end - overlap_start)
                    
                    if overlap > 0:
                        supports.append((self.brickToIndex[brick_below.coordinate], overlap))
                
                self.supportDependencies[brick.coordinate] = (supports, brick_len)

    def _IsBrickFullySupported(self, coordinate: types.Coordinate, placed_bricks_bitset: int) -> bool:
        if coordinate.row == 0:
            return True
        
        cache_key = (coordinate, placed_bricks_bitset)
        if cache_key in self.supportCheckCache:
            return self.supportCheckCache[cache_key]
        
        supports, brick_len = self.supportDependencies[coordinate]
        
        supported_length = 0
        for brick_below_index, overlap in supports:
            if (placed_bricks_bitset >> brick_below_index) & 1:
                supported_length += overlap
        
        result = supported_length >= brick_len
        self.supportCheckCache[cache_key] = result
        return result
  
    def _CalculateReachableBricks(self, robot_position: types.Position) -> Set[types.Coordinate]:
        pos_key = (robot_position.x, robot_position.y)
        
        if pos_key not in self.reachabilityCache:
            reachable = set()
            for course in self.wall:
                for brick in course:
                    if self._CalculateBrickInReach(brick, robot_position):
                        reachable.add(brick.coordinate)
            self.reachabilityCache[pos_key] = reachable
        
        return self.reachabilityCache[pos_key]

    def _CalculateBrickInReach(self, brick: types.Brick, 
                    robot_position: types.Position) -> bool:
        
        brick_start = brick.position.x
        brick_len = (self.config.fullBrickDimension.length 
                    if brick.brickType == types.BrickType.FULL 
                    else self.config.halfBrickDimension.length)
        brick_end = brick_start + brick_len
        
        brick_bottom_y = brick.position.y
        brick_height = (self.config.fullBrickDimension.height 
                    if brick.brickType == types.BrickType.FULL 
                    else self.config.halfBrickDimension.height)
        brick_top_y = brick_bottom_y + brick_height

        h_reach_start = robot_position.x
        h_reach_end = robot_position.x + self.config.envelope.length
        v_reach_start = robot_position.y
        v_reach_end = robot_position.y + self.config.envelope.height
        
        h_in_reach = (brick_start >= h_reach_start and brick_end <= h_reach_end)
        v_in_reach = (brick_bottom_y >= v_reach_start and brick_top_y <= v_reach_end)

        return h_in_reach and v_in_reach

    def _CreateStretcherLayout(self):
        nr_of_courses: int = int(self.config.wall.height // self.config.courseHeight)
        self.wall = []
        self.totalBricks = 0

        def __AddBrick(brick_type: types.BrickType, row: int, column: int, x_pos: int) -> int:
            coordinate = types.Coordinate(row, column)
            position = types.Position(x_pos, row * self.config.courseHeight)
            course.append(types.Brick(coordinate, position, brick_type))
            
            self.totalWallArea += (self.config.fullBrickArea if brick_type == types.BrickType.FULL else self.config.halfBrickArea)
            
            return (self.config.halfBrickWithJointLength 
                    if brick_type == types.BrickType.HALF 
                    else self.config.fullBrickWithJointLength)
        
        for row in range(nr_of_courses):
            course: List[types.Brick] = []
            current_width: int = 0
            column: int = 0

            if row % 2 == 1:
                current_width += __AddBrick(types.BrickType.HALF, row, column, current_width)
                column += 1

            while current_width + self.config.fullBrickDimension.length <= self.config.wall.length:
                current_width += __AddBrick(types.BrickType.FULL, row, column, current_width)
                column += 1

            if current_width + self.config.halfBrickDimension.length <= self.config.wall.length:
                current_width += __AddBrick(types.BrickType.HALF, row, column, current_width)
                column += 1

            self.wall.append(course)
            self.totalBricks += len(course)
        
        index = 0
        for course in self.wall:
            for brick in course:
                self.brickToIndex[brick.coordinate] = index
                index += 1

        print(f"\n=== BRICK LAYOUT DEBUG ===")
        for row_idx in range(min(2, len(self.wall))):  # Just first 2 rows
            print(f"Row {row_idx}:")
            for brick in self.wall[row_idx]:
                brick_end = brick.position.x + (self.config.fullBrickDimension.length if brick.brickType == types.BrickType.FULL else self.config.halfBrickDimension.length)
                print(f"  Col {brick.coordinate.column}: x=[{brick.position.x}, {brick_end}], type={brick.brickType}")

def main():
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--config', 
                        type=str, 
                        required=True, 
                        help="Give a path to the toml config file"
                        )
    
    parser.add_argument('--bond_type', 
                        type=str.upper, 
                        choices=[t.value for t in types.BondType], 
                        required=True, 
                        help="Choose a bond type by name"
                        )

    args = parser.parse_args()
    profiler = cProfile.Profile()
    profiler.enable()
    wallVisualizer = WallVisualizer(args.config)
    wallVisualizer.RunInteractiveBuild(types.BondType(args.bond_type))
    profiler.disable()

    # Print stats
    stats = pstats.Stats(profiler)
    stats.sort_stats(SortKey.CUMULATIVE)
    stats.print_stats(20)
    stats.print_callers(20)
    stats.print_callees(20)
if __name__ == "__main__":
    main()

    