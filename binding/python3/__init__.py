from .biorbd import *
from ._version import __version__

try:
    from casadi import Function, MX
    def to_sx_func(name, func, *all_param):
        mx_param = []
        for p in all_param:
            if isinstance(p, MX):
                mx_param.append(p)

        func_evaluated = func(*all_param)
        if not isinstance(func_evaluated, MX):
            func_evaluated = func_evaluated.to_mx()
        return Function(name, mx_param, [func_evaluated]).expand()
except ModuleNotFoundError:
    pass
