from pathlib import Path

import numpy as np
import biorbd

#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen position (Q), velocity (Qdot) and acceleration (Qddot)
#     3. Compute the generalized forces (tau) at this state (inverse dynamics)
#     4. Print them to the console
#
# Please note that this example will work only with the Eigen backend
#


def main():
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Model(f"{current_file_dir}/../pyomecaman.bioMod")
    nq = model.nbQ()
    nqdot = model.nbQdot()
    nqddot = model.nbQddot()

    # Choose a position/velocity/acceleration to compute dynamics from
    Q = np.zeros((nq,))
    Qdot = np.zeros((nqdot,))
    Qddot = np.zeros((nqddot,))

    # Proceed with the inverse dynamics
    Tau = model.InverseDynamics(Q, Qdot, Qddot)

    # Print them to the console
    print(Tau.to_array())


if __name__ == "__main__":
    main()
