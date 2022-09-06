from typing import Protocol
import numpy as np


class Data(Protocol):
    def mean_markers(self, marker_names: tuple[str, ...]) -> np.ndarray:
        """
        Return the actual data of specific markers meaned down to an XYZ vector
        """


class DynamicModel(Protocol):
    pass
