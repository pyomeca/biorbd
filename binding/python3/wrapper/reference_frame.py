from collections import UserList
from typing import TYPE_CHECKING, Iterator

from .misc import BiorbdArray, to_biorbd_array_output, to_biorbd_array_input

if TYPE_CHECKING:
    from .biorbd_model import Biorbd


# TODO Naming should be local_frame and global_frame for a class named SegmentFrame
class ReferenceFrame:
    pass


class ReferenceFrameList(UserList):
    data: list[ReferenceFrame]
