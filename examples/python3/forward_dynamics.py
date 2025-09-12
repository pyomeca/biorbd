from pathlib import Path

import biorbd
import numpy as np


#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen position (q), velocity (qdot)
#     3. Compute the generalized acceleration (qddot) assuming a set of generalized forces (tau), i.e. forward dynamics
#     4. Print them to the console
#
# Please note that this example will work only with the Eigen backend (as np.arange is used to generate the input arrays).
#


def main():
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Biorbd(f"{current_file_dir}/../pyomecaman.bioMod")

    # Define some arbitrary joint positions, velocities, and torques
    q = np.arange(model.nb_q)
    qdot = np.arange(model.nb_qdot)
    tau = np.arange(model.nb_tau)

    # Compute the forward dynamics
    qddot = model.forward_dynamics(q, qdot, tau)

    # Print them to the console
    print(qddot)

    # Alternatively, one can directly access the internal model
    qddot_via_internal = model.internal_model.ForwardDynamics(q, qdot, tau)
    # And check that both methods give the same result
    print(qddot_via_internal.to_array())


if __name__ == "__main__":
    main()
