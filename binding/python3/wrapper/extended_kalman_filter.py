from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .biorbd_model import Biorbd
from .misc import BiorbdArray, to_biorbd_array_input, to_biorbd_array_output
from ..biorbd import (
    KalmanParam,
    KalmanReconsMarkers,
    GeneralizedCoordinates,
    GeneralizedVelocity,
    GeneralizedAcceleration,
    NodeSegment,
)


class ExtendedKalmanFilterMarkers:
    def __init__(self, model: "Biorbd", frequency: int, noise_factor: float = None, error_factor: float = None):

        input_param_parameters = {"frequency": frequency}
        if noise_factor is not None:
            input_param_parameters["noiseFactor"] = noise_factor
        if error_factor is not None:
            input_param_parameters["errorFactor"] = error_factor

        self._param = KalmanParam(**input_param_parameters)
        self._model = model
        self._kalman = KalmanReconsMarkers(self._model.internal, self._param)

        self._q = GeneralizedCoordinates(self._model.internal)
        self._qdot = GeneralizedVelocity(self._model.internal)
        self._qddot = GeneralizedAcceleration(self._model.internal)

    def reconstruct_frame(self, markers: BiorbdArray):
        markers_kalman = [NodeSegment(m) for m in to_biorbd_array_input(markers).T]
        self._kalman.reconstructFrame(self._model.internal, markers_kalman, self._q, self._qdot, self._qddot)

        q = to_biorbd_array_output(self._q)
        qdot = to_biorbd_array_output(self._qdot)
        qddot = to_biorbd_array_output(self._qddot)
        return q, qdot, qddot

    def reconstruct_frames(self, all_markers: list[BiorbdArray]):
        for markers in all_markers:
            yield self.reconstruct_frame(markers)
