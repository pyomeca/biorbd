import numpy as np
from . import biorbd  # This is created while installing using CMake
from .biorbd import *
from ._version import __version__
from .surface_max_torque_actuator import *
from .rigid_body import *
from .utils import *


# If we are using the Eigen backend, returns are np.arrays, if we are using CasADi backend, returns are MX
# So declare an "Array" typedef that will be np.array or MX depending on the backend
if biorbd.currentLinearAlgebraBackend() == 1:
    from casadi import MX

    BiorbdArray = MX
else:
    BiorbdArray = np.ndarray


# Declare a method that converts output to BiorbdArray
def _to_biorbd_array(x):
    if biorbd.currentLinearAlgebraBackend() == 1:
        return x.to_mx()
    return x.to_array()


class Biorbd:
    def __init__(self, path):
        self._model = biorbd.Model(path)

    @property
    def internal_model(self) -> biorbd.Model:
        """
        Get the internal model of the Biorbd instance.

        Returns
        -------
        The internal model. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model

    @property
    def dof_names(self) -> list[str]:
        """
        Get the names of the degrees of freedom in the model.

        Returns
        -------
        A list of degree of freedom names
        """
        return [dof.to_string() for dof in self._model.nameDof()]

    @property
    def nb_q(self) -> int:
        """
        Get the number of generalized coordinates in the model.

        Returns
        -------
        The number of generalized coordinates
        """
        return self._model.nbQ()

    @property
    def nb_qdot(self) -> int:
        """
        Get the number of generalized velocities in the model.

        Returns
        -------
        The number of generalized velocities
        """
        return self._model.nbQdot()

    @property
    def nb_tau(self) -> int:
        """
        Get the number of generalized torques in the model.

        Returns
        -------
        The number of generalized torques
        """
        return self._model.nbGeneralizedTorque()

    def forward_dynamics(self, q: BiorbdArray, qdot: BiorbdArray, tau: BiorbdArray) -> BiorbdArray:
        """
        Perform forward dynamics on the model.

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities
        tau: BiorbdArray
            Generalized forces

        Returns
        -------
        Generalized accelerations
        """
        return _to_biorbd_array(self._model.ForwardDynamics(q, qdot, tau))

    def inverse_dynamics(self, q: BiorbdArray, qdot: BiorbdArray, qddot: BiorbdArray) -> BiorbdArray:
        """
        Perform inverse dynamics on the model.

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities
        qddot: BiorbdArray
            Generalized accelerations

        Returns
        -------
        Generalized forces
        """
        return _to_biorbd_array(self._model.InverseDynamics(q, qdot, qddot))


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
