import heapq
import tomllib
import argparse
import wall_visualizer_types as types
from typing import Dict, List, Set
from dataclasses import dataclass
import cProfile
import pstats

# These PODs are for the Dijkstra's algo
@dataclass(frozen=True)
class State:
    robotPosition: types.Position
    placedBricks: frozenset

    def __hash__(self):
        return hash((self.robotPosition.x, self.robotPosition.y, self.placedBricks))
    
    def __eq__(self, other):
        return (self.robotPosition.x == other.robotPosition.x and 
                self.robotPosition.y == other.robotPosition.y and 
                self.placedBricks == other.placedBricks)


@dataclass
class Node:
    cost: int
    state: State
    currentStrideId: int
    brickHistory: List[types.Brick]

    def __lt__(self, other):
        return self.cost < other.cost

class WallVisualizer:
    def __init__(self, config_file: str):
        self.config = self.LoadConfig(config_file)
        self.wall: List[List[types.Brick]] = []
        self.buildOrder: List[types.Brick] = []
        self.builtBricks: Set[types.Coordinate] = set()
        self.totalBricks: int = 0
        self.totalCost: int = 0
        self.totalStrides: int = 0 
        self.reachability_cache = {}

    def LoadConfig(self, file_name) -> types.Config:
        with open(file_name, 'rb') as f:
            data = tomllib.load(f)

        return types.Config(
            wall = types.Wall(
                length = data["WALL"]["length_mm"],
                height = data["WALL"]["height_mm"],
                ),
            
            fullBrickDimension = types.BrickDimension(
                length = data["FULL_BRICK"]["length_mm"], 
                height = data["FULL_BRICK"]["height_mm"],
                width = data["FULL_BRICK"]["width_mm"],
                ),
            
            halfBrickDimension = types.BrickDimension(
                length = data["HALF_BRICK"]["length_mm"], 
                height = data["HALF_BRICK"]["height_mm"],
                width = data["HALF_BRICK"]["width_mm"],
                ),
            
            jointSize = types.JointSize(
                head = data["JOINT_SIZE"]["head_mm"], 
                bed = data["JOINT_SIZE"]["bed_mm"],
                ),

            envelope = types.RobotEnvelope(
                length = data["ENVELOPE"]["length_mm"], 
                height = data["ENVELOPE"]["height_mm"],
                ),
            )
    
    def _ClearTerminal(self):
        print("\x1b[H\x1b[2J\x1b[3J", end="", flush=True)
        
    def _Visualize(self)-> None:
        self._ClearTerminal()

        brick_stride_map = {}
        for brick in self.buildOrder:
            brick_stride_map[brick.coordinate] = brick.strideId

        for row in reversed(range(len(self.wall))):
            course = self.wall[row]
            line = ""
            
            for brick in course:
                brick_key = brick.coordinate
                stride_id = brick_stride_map.get(brick_key, 0)
                strideColor = types.Color.GetStrideColor(stride_id)
                if brick_key in self.builtBricks:
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
            if(brick.strideId == 3):
                print("stride id 3")
            
    def RunInteractiveBuild(self, bond_type: types.BondType) -> None:
        print("Visualizing for bond type \"{}\"".format(bond_type))
        self._CreateLayout(bond_type)
        success = self._OptimizeBuildOrder()
        '''
        if not success:
            print("Failed to find valid build order!")
            return
        
        print(f"Optimal strategy found: total cost = {self.totalCost}, strides = {self.totalStrides}")
        print("Press ENTER to place next brick, 'q' to quit")

        self.builtBricks = set()
        for idx, brick in enumerate(self.buildOrder):
            user_input = input(f"\n[{idx + 1}/{len(self.buildOrder)}] "
                               f"Place {brick.brickType.value} brick at row {brick.coordinate.row}, col {brick.coordinate.column}? ").strip().lower()
            if user_input == 'q':
                print("Build stopped")
                break
            else:
                self.builtBricks.add(types.Coordinate(brick.coordinate.row, brick.coordinate.column))
                self._Visualize()
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


        print(f"Total robot positions: {len(all_positions)}")
        print(f"Total bricks: {self.totalBricks}")
        
        initial_robot_position: types.Position = types.Position(0.0, 0.0)
        priority_queue = []
        
        heapq.heappush(priority_queue, Node(cost=0, 
                                            state=State(robotPosition=initial_robot_position, placedBricks=frozenset()), 
                                            currentStrideId=0, 
                                            brickHistory=[]))

        best_cost: Dict[State, int] = {}
        max_queue_size = 0

        while priority_queue:
            if len(priority_queue) > max_queue_size:
                max_queue_size = len(priority_queue)

            current_node = heapq.heappop(priority_queue)

            if current_node.state in best_cost and best_cost[current_node.state] < current_node.cost:
                continue

            best_cost[current_node.state] = current_node.cost
            '''
            if len(current_node.state.placedBricks) % 10 == 0:
                print(f"Progress: {len(current_node.state.placedBricks)}/{self.totalBricks}, "
                    f"Cost: {current_node.cost}, Queue: {len(priority_queue)}")
            '''
            if len(current_node.state.placedBricks) == self.totalBricks:

                self.buildOrder = current_node.brickHistory
                self.totalCost = current_node.cost
                self.totalStrides = len(set(brick.strideId for brick in current_node.brickHistory)) - 1

                stride_info = {}
                for brick in current_node.brickHistory:
                    if brick.strideId not in stride_info:
                        stride_info[brick.strideId] = []
                    stride_info[brick.strideId].append(brick)
                print("MAX QUEUE SIZE", max_queue_size)
                print(f"\n=== SOLUTION DETAILS ===")
                for stride_id in sorted(stride_info.keys()):
                    bricks = stride_info[stride_id]
                    if bricks:
                        cols = sorted(set(b.coordinate.column for b in bricks))
                        print(f"Stride {stride_id}: {len(bricks)} bricks, rows {sorted(set(b.coordinate.row for b in bricks))}, cols {cols}")
                    
                last_pos = None
                positions_used = []

                for brick in current_node.brickHistory:
                    # Find robot position by looking at which stride this is
                    # (We need to track this differently)
                    pass

                # Better approach: track in the stride info
                print(f"\n=== SOLUTION DETAILS ===")

                # Reconstruct positions from history
                current_pos = types.Position(0.0, 0.0)
                positions_used = [(0, current_pos)]

                for i, brick in enumerate(current_node.brickHistory):
                    if i > 0 and brick.strideId != current_node.brickHistory[i-1].strideId:
                        # Stride changed, need to figure out new position
                        # We can infer this by checking which position can reach this brick
                        for pos in all_positions:
                            if self._IsBrickInReach(brick, pos):
                                positions_used.append((brick.strideId, pos))
                                break

                for stride_id, pos in positions_used:
                    print(f"Stride {stride_id}: Robot at ({pos.x}, {pos.y})")
                return True

            # DEBUG: Track what's happening
            placeable_bricks = 0
            reachable_from_current = 0
            needs_move = 0
            
            for course in self.wall:
                for brick in course:
                    if brick.coordinate in current_node.state.placedBricks:
                        continue
                    if not self._IsBrickFullySupported(brick, current_node.state.placedBricks):
                        continue
                    placeable_bricks += 1

                    
                    
                    if self._IsBrickInReach(brick, current_node.state.robotPosition):

                        reachable_from_current += 1
                        new_state = State(
                            robotPosition=current_node.state.robotPosition,
                            placedBricks=frozenset(current_node.state.placedBricks | {brick.coordinate})
                        )
                        
                        if new_state not in best_cost or current_node.cost < best_cost[new_state]:
                            best_cost[new_state] = current_node.cost
                            new_node = Node(
                                cost=current_node.cost,
                                state=new_state,
                                currentStrideId=current_node.currentStrideId,
                                brickHistory=current_node.brickHistory + [types.Brick(brick.coordinate, brick.position, brick.brickType, current_node.currentStrideId)],
                            )
                            heapq.heappush(priority_queue, new_node)
                    
                    else:
                        needs_move += 1
                        # Find positions that can reach this brick
                        found_any = False
                        for new_pos in all_positions:
                            if (abs(new_pos.x - current_node.state.robotPosition.x) < 0.01 and 
                                abs(new_pos.y - current_node.state.robotPosition.y) < 0.01):
                                continue
                            
                            if not self._IsBrickInReach(brick, new_pos):
                                continue

                            found_any = True
                            movement_cost = 1
                            new_stride_id = current_node.currentStrideId + 1
                            new_cost = current_node.cost + movement_cost
                            new_state = State(
                                robotPosition=new_pos,
                                placedBricks=frozenset(current_node.state.placedBricks | {brick.coordinate})
                            )
                            
                            if new_state not in best_cost or new_cost < best_cost[new_state]:
                                best_cost[new_state] = new_cost
                                new_node = Node(
                                    cost=new_cost,
                                    state=new_state,
                                    currentStrideId=new_stride_id,
                                    brickHistory=current_node.brickHistory + [types.Brick(brick.coordinate, brick.position, brick.brickType, new_stride_id)],
                                )
                                heapq.heappush(priority_queue, new_node)
                        
                        if not found_any:
                            print(f"ERROR: No position can reach brick at row {brick.coordinate.row}, col {brick.coordinate.column}")


        return False

    def _GetAllPossibleRobotPositions(self) -> List[types.Position]:
        positions = []
            
        h_step = int(self.config.halfBrickDimension.length)
        v_step = int(self.config.envelope.height // 32)
        
        h_start = 0
        h_end = int(self.config.wall.length)
        
        v_start = 0
        v_end = int(self.config.wall.height)
        
        print(f"DEBUG Position generation:")
        print(f"  H: {h_start} to {h_end} step {h_step}")
        print(f"  V: {v_start} to {v_end} step {v_step}")
        
        for h_pos in range(h_start, h_end, h_step):
            for v_pos in range(v_start, v_end, v_step):
                positions.append(types.Position(float(h_pos), float(v_pos)))
        
        print(f"  Sample positions: {positions[:10]}")
        print(f"  Unique Y values: {sorted(set(p.y for p in positions))}")
        
        return positions
        
        
    def _IsBrickFullySupported(self, brick: types.Brick, 
                            placed_bricks: frozenset) -> bool:
        #print("Brick ", brick.position, len(placed_bricks))
        if brick.coordinate.row == 0:
            return True

        brick_start = brick.position.x
        brick_len = (self.config.fullBrickDimension.length 
                    if brick.brickType == types.BrickType.FULL 
                    else self.config.halfBrickDimension.length)
        brick_end = brick_start + brick_len
        
        course_below = self.wall[brick.coordinate.row - 1]
        
        supported_length = 0.0
        for brick_below in course_below:
            if brick_below.coordinate not in placed_bricks:
                continue
            
            below_start = brick_below.position.x
            below_len = (self.config.fullBrickDimension.length 
                        if brick_below.brickType == types.BrickType.FULL
                        else self.config.halfBrickDimension.length)
            below_end = below_start + (below_len + self.config.jointSize.head)
            
            overlap_start = max(brick_start, below_start)
            overlap_end = min(brick_end, below_end)
            overlap = max(0, overlap_end - overlap_start)
            
            supported_length += overlap


        return supported_length >= brick_len
    
    def _CalculateReachableBricks(self, robot_position: types.Position) -> Set[types.Coordinate]:
        """Get all bricks reachable from a position (cached)"""
        pos_key = (robot_position.x, robot_position.y)
        
        if pos_key not in self.reachability_cache:
            reachable = set()
            for course in self.wall:
                for brick in course:
                    if self._CalculateBrickInReach(brick, robot_position):
                        reachable.add(brick.coordinate)
            self.reachability_cache[pos_key] = reachable
        
        return self.reachability_cache[pos_key]

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
    
    def _IsBrickInReach(self, brick: types.Brick, robot_position: types.Position):
        pos_key = (robot_position.x, robot_position.y)
        reachable_coords = self.reachability_cache[pos_key]
        if brick.coordinate in reachable_coords:
            return True
        return False
            

    def _CreateStretcherLayout(self):
        nr_of_courses: int = int(self.config.wall.height // self.config.courseHeight)
        self.wall = []
        self.totalBricks = 0

        for row in range(nr_of_courses):
            course: List[types.Brick] = []
            current_width: float = 0.0
            column: int = 0


            if row % 2 == 1:
                brickCoordinate = types.Coordinate(row, column)
                brickPosition = types.Position(current_width, row * self.config.courseHeight)
                course.append(types.Brick(brickCoordinate,
                                          brickPosition,
                                          types.BrickType.HALF))
                current_width += self.config.halfBrickWithJointLength
                column += 1

            while current_width + self.config.fullBrickDimension.length <= self.config.wall.length:
                brickCoordinate = types.Coordinate(row, column)
                brickPosition = types.Position(current_width, row * self.config.courseHeight)
                course.append(types.Brick(brickCoordinate,
                                          brickPosition,
                                          types.BrickType.FULL))
                current_width += self.config.fullBrickWithJointLength
                column += 1

            if current_width + self.config.halfBrickDimension.length <= self.config.wall.length:
                brickCoordinate = types.Coordinate(row, column)
                brickPosition = types.Position(current_width, row * self.config.courseHeight)
                course.append(types.Brick(brickCoordinate,
                                          brickPosition,
                                          types.BrickType.HALF))
                column += 1

            self.wall.append(course)
            self.totalBricks += len(course)

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
    stats.strip_dirs()
    stats.sort_stats('cumulative')
    stats.print_stats(20)

if __name__ == "__main__":
    main()

    