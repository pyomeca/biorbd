import numpy as np
import ezc3d


class C3dData:
    """
    Implementation of the `Data` protocol from model_creation
    """
    def __init__(self, c3d_path, first_frame: int = 0, last_frame: int = -1):
        self.first_frame = first_frame
        self.last_frame = last_frame
        self.data = ezc3d.c3d(c3d_path)

        if self.data["data"]["points"].shape[2] == 1 and self.last_frame == -1:
            self.last_frame = 2  # This is a bug otherwise since data[:, :, 0:-1] returns nothing

    def mean_markers(self, marker_names: tuple[str, ...]) -> np.ndarray:
        return self._to_meter(
            np.mean(
                np.nanmean(
                    self.data["data"]["points"][
                        :, self._indices_in_c3d(marker_names), self.first_frame : self.last_frame
                    ],
                    axis=2,
                ),
                axis=1,
            )
        )

    def _indices_in_c3d(self, from_markers: tuple[str, ...]) -> tuple[int, ...]:
        return tuple(self.data["parameters"]["POINT"]["LABELS"]["value"].index(n) for n in from_markers)

    def _to_meter(self, data: np.array) -> np.array:
        units = self.data["parameters"]["POINT"]["UNITS"]["value"]
        factor = 1000 if len(units) > 0 and units[0] == "mm" else 1

        data /= factor
        data[3] = 1
        return data
