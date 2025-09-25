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
    model = biorbd.Biorbd(f"{current_file_dir}/../pyomecaman.bioMod")

    # Choose a position/velocity/acceleration to compute dynamics from
    q = [0] * model.nb_q
    qdot = [0] * model.nb_qdot
    qddot = [0] * model.nb_qddot

    # Proceed with the inverse dynamics
    tau = model.inverse_dynamics(q, qdot, qddot)

    # Print them to the console
    print(f"Inverse dynamics tau: {tau}")

    # # Alternatively, one can directly access the internal model
    # tau_via_internal = model.internal.InverseDynamics(np.array(q), np.array(qdot), np.array(qddot))
    # # And check that both methods give the same result
    # print(tau_via_internal.to_array())

    # One can also add external forces to the model
    model.external_force_set.add(  # Add an external force of 100N in Z direction at the origin of the first segment
        segment_name=model.segments[0].name, force=[0, 0, 100], point_of_application=[0, 0, 0]
    )

    tau_with_external_forces = model.inverse_dynamics(q, qdot, qddot)
    print(f"Inverse dynamics with external forces tau: {tau_with_external_forces}")

    # But they can be ignored if needed
    tau_ignore_external_forces = model.inverse_dynamics(q, qdot, qddot, ignore_external_forces=True)
    print(f"Inverse dynamics without external forces tau: {tau_ignore_external_forces}")


if __name__ == "__main__":
    main()
