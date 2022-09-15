from typing import Callable

from .biomechanical_model_real import BiomechanicalModelReal
from .mesh_real import MeshReal
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class Mesh:
    def __init__(
        self,
        functions: tuple[Callable | str, ...],
    ):
        """
        This is a pre-constructor for the MeshReal class. It allows to create a generic model by marker names

        Parameters
        ----------
        functions
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the marker with.
            If a str is provided, the position of the corresponding marker is used
        """
        self.functions = tuple((lambda m, bio, name=f: m[name]) if isinstance(f, str) else f for f in functions)

    def to_mesh(
        self, data: Data, kinematic_chain: BiomechanicalModelReal, parent_scs: SegmentCoordinateSystemReal = None
    ) -> MeshReal:
        return MeshReal.from_data(
            data,
            self.functions,
            kinematic_chain,
            parent_scs,
        )
