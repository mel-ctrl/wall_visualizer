from enum import Enum, StrEnum
from dataclasses import dataclass


class Color:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    ORANGE = '\033[38;5;208m'
    PURPLE = '\033[38;5;129m'
    PINK = '\033[38;5;213m'
    RESET = '\033[0m'
    GRAY = '\033[90m'
    RESET = '\033[0m'

    @staticmethod
    def GetStrideColor(stride_id: int) -> str:
        colors = [
            Color.RED,
            Color.GREEN, 
            Color.BLUE,
            Color.YELLOW,
            Color.MAGENTA,
            Color.CYAN,
            Color.ORANGE,
            Color.PURPLE,
            Color.PINK,
        ]
        return colors[stride_id % len(colors)]


class BrickType(StrEnum):
    FULL = "FULL"
    HALF = "HALF"

class BondType(StrEnum):
    STRETCHER = "STRETCHER"
    ENGLISH_CROSS_BOND = "ENGLISH_CROSS_BOND"
    WILD_BOND = "WILD_BOND"

@dataclass(frozen=True)
class Coordinate:
    row: int = 0
    column: int = 0

    def __eq__(self, other):
        return self.row == other.row and self.column == other.column

@dataclass(frozen=True)
class Position:
    x: int = 0
    y: int = 0

@dataclass
class Brick:
    coordinate: Coordinate
    position: Position
    brickType: BrickType
    strideId: int = 0

@dataclass
class BrickDimension:
    length: int = 0
    height: int = 0
    width: int = 0

@dataclass
class RobotEnvelope:
    length: int = 0
    height: int = 0

@dataclass
class JointSize:
    head: int = 0
    bed: int = 0

@dataclass
class Wall:
    length: int = 0
    height: int = 0

@dataclass
class Action:
    move: Coordinate
    build: Brick 

@dataclass
class Config:
    wall: Wall
    fullBrickDimension: BrickDimension
    halfBrickDimension: BrickDimension
    jointSize: JointSize
    envelope: RobotEnvelope


    @property
    def courseHeight(self):
        return self.fullBrickDimension.height + self.jointSize.bed
    
    @property
    def fullBrickWithJointLength(self):
        return self.fullBrickDimension.length + self.jointSize.head

    @property
    def halfBrickWithJointLength(self):
        return self.halfBrickDimension.length + self.jointSize.head
    
    @property
    def fullBrickArea(self):
        return self.fullBrickDimension.length * self.fullBrickDimension.height
    
    @property
    def halfBrickArea(self):
        return self.halfBrickDimension.length * self.halfBrickDimension.height


