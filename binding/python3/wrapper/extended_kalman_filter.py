from typing import TYPE_CHECKING, Generator

if TYPE_CHECKING:
    from .biorbd_model import Biorbd
from .misc import BiorbdArray, BiorbdScalar, to_biorbd_array_input, to_biorbd_array_output

try:
    from ..biorbd import (
        KalmanParam,
        KalmanReconsMarkers,
        GeneralizedCoordinates,
        GeneralizedVelocity,
        GeneralizedAcceleration,
        NodeSegment,
    )

    has_extended_kalman_filter = True
except ImportError:
    has_extended_kalman_filter = False


class ExtendedKalmanFilterMarkers:
    def __init__(
        self,
        model: "Biorbd",
        frequency: int,
        noise_factor: BiorbdScalar = None,
        error_factor: BiorbdScalar = None,
        q_init: BiorbdArray = None,
        qdot_init: BiorbdArray = None,
        qddot_init: BiorbdArray = None,
    ):
        if not has_extended_kalman_filter:
            raise RuntimeError(
                "In order to use ExtendedKalmanFilterMarkers, biorbd must be compiled with MODULE_KALMAN=ON"
            )

        input_param_parameters = {"frequency": frequency}
        if noise_factor is not None:
            input_param_parameters["noiseFactor"] = noise_factor
        if error_factor is not None:
            input_param_parameters["errorFactor"] = error_factor

        self._param = KalmanParam(**input_param_parameters)
        self._model = model
        self._kalman = KalmanReconsMarkers(self._model.internal, self._param)
        self._kalman.setInitState(
            to_biorbd_array_input(q_init), to_biorbd_array_input(qdot_init), to_biorbd_array_input(qddot_init)
        )

        self._q = GeneralizedCoordinates(self._model.internal)
        self._qdot = GeneralizedVelocity(self._model.internal)
        self._qddot = GeneralizedAcceleration(self._model.internal)

    def reconstruct_frame(self, markers: BiorbdArray):
        markers_kalman = [NodeSegment(m) for m in to_biorbd_array_input(markers, enforce_iterable=True).T]
        if len(markers_kalman) != self._model.internal.nbTechnicalMarkers():
            raise RuntimeError(
                f"Number of markers provided ({len(markers_kalman)}) does not match the number of technical markers in "
                f"the model ({self._model.internal.nbTechnicalMarkers()})."
            )

        self._kalman.reconstructFrame(self._model.internal, markers_kalman, self._q, self._qdot, self._qddot)

        q = to_biorbd_array_output(self._q)
        qdot = to_biorbd_array_output(self._qdot)
        qddot = to_biorbd_array_output(self._qddot)
        return q, qdot, qddot

    def reconstruct_frames(
        self, all_markers: list[BiorbdArray]
    ) -> Generator[tuple[BiorbdArray, BiorbdArray, BiorbdArray], None, None]:
        for markers in all_markers:
            yield self.reconstruct_frame(markers)
