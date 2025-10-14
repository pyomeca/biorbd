brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd: {e}")
    pass

try:
    import biorbd_casadi
    from casadi import MX

    brbd_to_test.append(biorbd_casadi)
except ModuleNotFoundError as e:
    pass

if not brbd_to_test:
    raise RuntimeError("No biorbd version could be imported")

import numpy as np


def evaluate(brbd, func, **kwargs):
    if brbd.backend == brbd.EIGEN3:
        if not kwargs:
            return func
        else:
            return func(**kwargs)
    elif brbd.backend == brbd.CASADI:
        symbolic_keys = ["q", "qdot", "qddot", "tau"]
        symbolics = {
            key: MX.sym(
                key, np.array(value).shape[0], np.array(value).shape[1] if len(np.array(value).shape) > 1 else 1
            )
            for key, value in kwargs.items()
            if key in symbolic_keys
        }
        non_symbolics = {key: value for key, value in kwargs.items() if key not in symbolic_keys}
        used_kwargs = {key: value for key, value in kwargs.items() if key in symbolic_keys}
        value = brbd.to_casadi_func("temp", func, **symbolics, **non_symbolics)(*used_kwargs.values())
        if isinstance(value, dict):
            value_tp = []
            for v in value.values():
                v = np.array(v)
                if v.shape[0] == 1 and v.shape[1] == 1:
                    value_tp.append(v[0, 0])
                elif v.shape[1] == 1:
                    value_tp.append(v[:, 0])
                elif v.shape[0] == 1:
                    value_tp.append(v[0, :])
                else:
                    value_tp.append(v)
            return value_tp[0] if len(value_tp) == 1 else value_tp
        else:
            value = np.array(value)
            if value.shape[0] == 1 and value.shape[1] == 1:
                value = value[0, 0]
            elif value.shape[1] == 1:
                value = value[:, 0]
            elif value.shape[0] == 1:
                value = value[0, :]
            return value
    else:
        raise RuntimeError("Unknown backend")
