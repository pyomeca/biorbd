from typing import Callable

import numpy as np

from .biomechanical_model_real import BiomechanicalModelReal
from .mesh_file_real import MeshFileReal
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class MeshFile:
    def __init__(
        self,
        mesh_file_name: str,
        mesh_color: np.ndarray[float] | list[float] | tuple[float] = None,
        scaling_function: Callable = None,
        rotation_function: Callable = None,
        translation_function: Callable = None,
    ):
        """
        This is a pre-constructor for the MeshFileReal class. It allows to create a generic model by marker names

        Parameters
        ----------
        mesh_file_name
            The name of the mesh file
        mesh_color
            The color the mesh should be displayed in (RGB)
        scaling_function
            The function that defines the scaling of the mesh
        rotation_function
            The function that defines the rotation of the mesh
        translation_function
            The function that defines the translation of the mesh
        """
        self.mesh_file_name = mesh_file_name
        self.mesh_color = mesh_color
        self.scaling_function = scaling_function
        self.rotation_function = rotation_function
        self.translation_function = translation_function

    def to_mesh_file(
        self,
        data: Data,
    ) -> MeshFileReal:
        return MeshFileReal.from_data(
            data,
            self.mesh_file_name,
            self.mesh_color,
            self.scaling_function,
            self.rotation_function,
            self.translation_function,
        )
