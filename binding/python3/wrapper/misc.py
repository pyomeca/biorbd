from typing import Any

import numpy as np

from ..biorbd import CASADI as BIORBD_CASADI, EIGEN3 as BIORBD_EIGEN3, currentLinearAlgebraBackend, Vector

# Additional imports for casadi backend
if currentLinearAlgebraBackend() == BIORBD_CASADI:
    from casadi import MX, SX, DM


# If we are using the Eigen backend, returns are np.arrays, if we are using CasADi backend, returns are MX
# So declare an "Array" typedef that will be np.array or MX depending on the backend
if currentLinearAlgebraBackend() == BIORBD_CASADI:
    type BiorbdArray = MX
    type BiorbdScalar = MX
else:
    type BiorbdArray = np.ndarray
    type BiorbdScalar = float


# Declare a method that converts output to BiorbdArray
def to_biorbd_array_output(x: Any) -> BiorbdArray:
    if currentLinearAlgebraBackend() == BIORBD_EIGEN3:
        if isinstance(x, (int, float, np.ndarray)):
            return x
        return x.to_array()
    elif currentLinearAlgebraBackend() == BIORBD_CASADI:
        return x.to_mx()
    else:
        raise NotImplementedError("Unknown backend")


def to_biorbd_array_input(x: BiorbdArray) -> Any:
    if x is None:
        return None

    if isinstance(x, Vector):
        return x

    if currentLinearAlgebraBackend() == BIORBD_EIGEN3:
        if isinstance(x, np.ndarray) or isinstance(x, (int, float)):
            return x
        return np.array(x, dtype=float)

    elif currentLinearAlgebraBackend() == BIORBD_CASADI:
        if isinstance(x, (MX, SX, DM)):
            return x
        try:
            return MX(x)
        except NotImplementedError:
            # This happens when we try to convert a list of lists. When sending DM, it should be available to the user
            tp = np.array(x)
            return MX(tp)
    else:
        raise NotImplementedError("Unknown backend")
