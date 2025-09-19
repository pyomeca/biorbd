from typing import TYPE_CHECKING, Generator

if TYPE_CHECKING:
    from .biorbd_model import Biorbd
from .misc import BiorbdArray, to_biorbd_array_input, to_biorbd_array_output
from ..biorbd import GeneralizedCoordinates, GeneralizedVelocity, GeneralizedTorque

try:
    from ..biorbd import StaticOptimization as StaticOptimizationBiorbd

    has_static_optimization = True
except ImportError:
    has_static_optimization = False


class StaticOptimization:
    def __init__(self, model: "Biorbd"):
        if not has_static_optimization:
            raise RuntimeError("In order to use StaticOptimization, biorbd must be compiled with STATIC_OPTIM=ON")

        self._model = model

    def perform_frames(self, all_q: BiorbdArray, all_qdot: BiorbdArray, all_tau: BiorbdArray) -> Generator[BiorbdArray]:
        q = [GeneralizedCoordinates(to_biorbd_array_input(q)) for q in all_q]
        qdot = [GeneralizedVelocity(to_biorbd_array_input(q)) for q in all_qdot]
        tau = [GeneralizedTorque(to_biorbd_array_input(t)) for t in all_tau]

        optim = StaticOptimizationBiorbd(self._model.internal, q, qdot, tau)
        optim.run()
        muscles_activations = optim.finalSolution()

        for activation in muscles_activations:
            yield to_biorbd_array_output(activation)
