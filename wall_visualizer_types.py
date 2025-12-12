from enum import Enum, StrEnum
from dataclasses import dataclass

class BrickType(StrEnum):
    FULL = "FULL"
    HALF = "HALF"

class BondType(StrEnum):
    STRETCHER = "STRETCHER"
    ENGLISH_CROSS_BOND = "ENGLISH_CROSS_BOND"
    WILD_BOND = "WILD_BOND"

@dataclass(frozen=True)
class Coordinate:
    row: int
    column: int

@dataclass
class Brick:
    coordinate: Coordinate
    brickType: BrickType

@dataclass
class BrickDimension:
    length: int
    height: int
    width: int

@dataclass
class RobotEnvelope:
    length: int
    height: int

@dataclass
class JointSize:
    head: int
    bed: int

@dataclass
class Wall:
    length: int
    height: int

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
        return self.halfBrickDimension.length
