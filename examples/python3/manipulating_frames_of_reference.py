from pathlib import Path

import numpy as np
from biorbd import Biorbd

#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen position (Q)
#     3. Compute the position of the world frame of each segment at that position
#     4. Print them to the console
#     5. Change (not supported yet) the local frame of one segment and see how its world frame is also updated
#
# Please note that this example will work only with the Eigen backend
#


def main(show: bool = True):
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = Biorbd(f"{current_file_dir}/../pyomecaman.bioMod")

    # Simulate a nice standing position
    q = [0, 0, -0.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0]

    # Compute the reference frames of each segment at that position
    # For clarity sake, just print the first segment, in normal use, one would probably want to use all segments
    frame = model.segments[0].frame
    print(f"Local frame of the segment {frame.name}:\n{frame.local}\n")
    print(f"World frame of the segment {frame.name} at q:\n{frame(q)}\n")
    # Once the model's kinematics have been updated, segment.frame.world will give the same result
    # Please note that the model can be preupdated by calling model.update_kinematics(q) or by calling segment.markers(q)
    print(f"World frame of the segment {frame.name} at q:\n{frame.world}\n")

    # We can extract some useful information from the frame
    print(f"Rotation matrix of the segment {frame.name} at q:\n{frame.local_rotation}\n")
    print(f"Rotation matrix of the segment {frame.name} at q:\n{frame.local_rotation_as_euler("xyz")}\n")
    print(f"Translation vector of the segment {frame.name} at q:\n{frame.local_translation}\n")

    print(f"World rotation matrix of the segment {frame.name} at q:\n{frame.world_rotation}\n")
    print(f"World rotation matrix of the segment {frame.name} at q:\n{frame.world_rotation_as_euler("xyz")}\n")
    print(f"World translation vector of the segment {frame.name} at q:\n{frame.world_translation}\n")

    # # This is not supported yet: Move one local frame to show that its world frame is also updated
    # frame = model.segments[0].frame
    # frame.local = [[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    # print(f"New local frame of the segment {model.segments[0].name}:\n{frame.local}\n")
    # print(f"New world frame of the segment {model.segments[0].name} at q:\n{frame(q)}\n")


if __name__ == "__main__":
    main()
