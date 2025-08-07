import numpy as np
from . import biorbd  # This is created while installing using CMake
from .biorbd import *
from ._version import __version__
from .surface_max_torque_actuator import *
from .rigid_body import *
from .utils import *

# TODO: This below
class GeneralizedCoordinates():
    def __init__(self, q):
        """
        Initialize the GeneralizedCoordinates with a numpy array.
        :param q: Generalized coordinates as a numpy array
        """
        self._q = np.array(q, dtype=float)

    @property
    def values(self):
        """
        Get the values of the generalized coordinates.
        :return: The generalized coordinates as a numpy array
        """
        return self._q

class MuscleThelen():
    def __init__(self, muscle):
        """
        Initialize the MuscleThelen with a muscle object.
        :param muscle: The muscle object
        """
        self._muscle = muscle

    @property
    def internal_muscle(self):
        """
        Get the internal muscle of the MuscleThelen instance.
        :return: The internal muscle
        """
        self.MUSCLE_TYPE = {
            "biorbd.MuscleThelen": self._muscle,
            "biorbd.Muscle": self._muscle,
            "biorbd.InternalForces.Muscles.MuscleThelen": self._muscle,
            "biorbd.InternalForces.Muscles.Muscle": self._muscle
        }
        return self._muscle.muscles() -> "biorbd.Muscle" -> biorbd.MuscleThelen

class Biorbd():
    def __init__(self, path):
        self._model = ...

    @property
    def internal_model(self):
        """
        Get the internal model of the Biorbd instance.
        :return: The internal model
        """
        return self._model

    def dof_names(self) -> list[str]:
        """
        Get the names of the degrees of freedom in the model.
        :return: A list of degree of freedom names
        """
        return [self._model.dofName(i).to_string() for i in range(self._model.nbQ())]

    def muscle_states(self, q: np.array, qdot: np.array, tau: np.array):

    def forward_dynamics(self, q: np.array, qdot, tau, gravity=None):
        """
        Perform forward dynamics on the model.
        :param q: Generalized coordinates
        :param qdot: Generalized velocities
        :param tau: Generalized forces
        :param gravity: Gravity vector (optional)
        :return: Generalized accelerations
        """
        if gravity is None:
            gravity = np.zeros((self._model.nbQ(), 1))
        return self._model.ForwardDynamics(q, qdot, tau)

if biorbd.currentLinearAlgebraBackend() == 1:
    from casadi import Function, MX, SX, horzcat

    def to_casadi_func(name, func, *all_param, expand=True):
        cx_param = []
        for p in all_param:
            if isinstance(p, (MX, SX)):
                cx_param.append(p)

        if isinstance(func, (MX, SX, Function)):
            func_evaluated = func
        else:
            func_evaluated = func(*all_param)
            if isinstance(func_evaluated, (list, tuple)):
                func_evaluated = horzcat(*[val if isinstance(val, MX) else val.to_mx() for val in func_evaluated])
            elif not isinstance(func_evaluated, MX):
                func_evaluated = func_evaluated.to_mx()
        func = Function(name, cx_param, [func_evaluated])
        return func.expand() if expand else func
