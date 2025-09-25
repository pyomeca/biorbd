from . import biorbd  # This is created while installing using CMake
from .biorbd import *
from ._version import __version__
from .surface_max_torque_actuator import *
from .rigid_body import *
from .utils import *
from .wrapper import *
from .wrapper import has_static_optimization, has_extended_kalman_filter


if biorbd.currentLinearAlgebraBackend() == biorbd.CASADI:
    from casadi import MX, SX, Function, horzcat

    backend = biorbd.CASADI

    def to_casadi_func(name, func, *all_param, expand=True, **kwargs):
        cx_param = []
        for p in all_param:
            if isinstance(p, (MX, SX)):
                cx_param.append(p)
        for value in kwargs.values():
            if isinstance(value, (MX, SX)):
                cx_param.append(value)

        if isinstance(func, (MX, SX, Function)):
            func_evaluated = [func]
        else:
            func_evaluated = func(*all_param, **kwargs)
            if isinstance(func_evaluated, (list, tuple)):
                func_evaluated = [val if isinstance(val, MX) else val.to_mx() for val in func_evaluated]
            elif not isinstance(func_evaluated, MX):
                func_evaluated = [func_evaluated.to_mx()]
            else:
                func_evaluated = [func_evaluated]
        func = Function(name, cx_param, func_evaluated)
        return func.expand() if expand else func

elif biorbd.currentLinearAlgebraBackend() == biorbd.EIGEN3:
    backend = biorbd.EIGEN3

    def to_casadi_func(name, func, *all_param, expand=True):
        raise RuntimeError("to_casadi_func is only available with the CASADI backend")

else:
    raise RuntimeError("Unknown backend")
