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
    print(f"Forward dynamics qddot: {qddot}")

    # # Alternatively, one can directly access the internal model
    # qddot_via_internal = model.internal.ForwardDynamics(q, qdot, tau)
    # # And check that both methods give the same result
    # print(qddot_via_internal.to_array())

    # If one wants to ignore the contacts defined in the model, they can do so as follows
    qddot_no_contact = model.forward_dynamics(q, qdot, tau, ignore_contacts=True)
    print(f"Forward dynamics without contacts qddot: {qddot_no_contact}")

    # The same can be done for external forces if any are defined in the model
    model.external_force_set.add(  # Add an external force of 100N in Z direction at the origin of the first segment
        segment_name=model.segments[0].name, force=[0, 0, 100], point_of_application=[0, 0, 0]
    )
    qddot_with_external_forces = model.forward_dynamics(q, qdot, tau)
    print(f"Forward dynamics with external forces qddot: {qddot_with_external_forces}")
    qddot_ignore_external_forces = model.forward_dynamics(q, qdot, tau, ignore_external_forces=True)
    print(f"Forward dynamics without external forces qddot: {qddot_ignore_external_forces}")


if __name__ == "__main__":
    main()
