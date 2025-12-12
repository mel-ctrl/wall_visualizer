import tomllib
import argparse
import wall_visualizer_types as types
from typing import List, Set
import os

class WallVisualizer:
    def __init__(self, config_file: str):
        self.config = self.LoadConfig(config_file)
        self.wall: List[List[types.Brick]] = []
        self.buildOrder: List[types.Brick] = []
        self.builtBricks: Set[types.Coordinate] = set()

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
        for row in reversed(range(len(self.wall))):
            course = self.wall[row]
            line = ""
            
            for brick in course:
                brick_key = brick.coordinate
                if brick_key in self.builtBricks:
                    if brick.brickType == types.BrickType.FULL:
                        line += "▓▓▓▓ "
                    else:
                        line += "▓▓ "
                else:
                    if brick.brickType == types.BrickType.FULL:
                        line += "░░░░ "
                    else:
                        line += "░░ "

            print(f"Row {row:2d} | {line}")
            
    def RunInteractiveBuild(self, bond_type: types.BondType) -> None:
        print("Visualizing for bond type \"{}\"".format(bond_type))
        self._CreateLayout(bond_type)
        self._OptimizeBuildOrder()

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

    def _OptimizeBuildOrder(self):
        #For now non-optimized (left to right from bottom up)
        self.buildOrder = []
        for course in self.wall:
            for brick in course:
                self.buildOrder.append(brick)


    def _CreateStretcherLayout(self):
        nr_of_courses: int = int(self.config.wall.height // self.config.courseHeight)
        self.wall = []

        for row in range(nr_of_courses):
            course: List[types.Brick] = []
            current_width: int = 0
            column: int = 0

            if row % 2 == 1:
                course.append(types.Brick(types.Coordinate(row, column), types.BrickType.HALF))
                current_width += self.config.halfBrickWithJointLength
                column += 1

            while current_width + self.config.fullBrickDimension.length <= self.config.wall.length:
                course.append(types.Brick(types.Coordinate(row, column), types.BrickType.FULL))
                current_width += self.config.fullBrickWithJointLength
                column += 1

            if current_width + self.config.halfBrickDimension.length <= self.config.wall.length:
                course.append(types.Brick(types.Coordinate(row, column), types.BrickType.HALF))

            self.wall.append(course)


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
    
    wallVisualizer = WallVisualizer(args.config)
    wallVisualizer.RunInteractiveBuild(types.BondType(args.bond_type))


if __name__ == "__main__":
    main()

    