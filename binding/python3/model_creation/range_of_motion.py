
from enum import Enum


class Ranges(Enum):
    Q = "Q"
    Qdot = "Qdot"


class RangeOfMotion:
    def __init__(self,
                 type: Ranges,
                 min_bound: list[float],
                 max_bound: list[float]):

        self.type = type
        self.min_bound = min_bound
        self.max_bound = max_bound

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        if self.type == Ranges.Q:
            out_string = f"\trangesQ \n"
        elif self.type == Ranges.Qdot:
            out_string = f"\trangesQdot \n"
        else:
            raise RuntimeError("RangeOfMotion's type must be Range.Q or Ranges.Qdot")

        for i_dof in range(len(self.min_bound)):
            out_string += f"\t\t{self.min_bound[i_dof]:0.5f}\t{self.max_bound[i_dof]:0.5f}\n"
        out_string += "\n"

        return out_string