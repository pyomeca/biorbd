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
    model = biorbd.Biorbd(f"{current_file_dir}/../cube.bioMod")

    # Segment on which the external force will be applied
    segment = model.segments[0]

    # Computing forward dynamics as reference
    q = [0] * model.nb_q
    qdot = [0] * model.nb_qdot
    tau = [0] * model.nb_tau
    ref_qddot = model.forward_dynamics(q=q, qdot=qdot, tau=tau)
    print(f"Reference qddot (without external forces): {ref_qddot}")

    # Add an external force to the model that counteracts gravity
    model.external_force_set.add(
        segment_name=segment.name,  # Name of the segment where the force is applied
        force=[0, 0, 9.81 * segment.mass],  # Force vector in Newtons
        point_of_application=[0, 0, 0],  # Point of application in the local frame of the segment
    )

    # Compute the forward dynamics
    qddot = model.forward_dynamics(q, qdot, tau)
    print(f"qddot (with external forces): {qddot}")

    # Add an extra external force to the model that pushes upward even more
    model.external_force_set.add(segment_name=segment.name, force=[0, 0, 1], point_of_application=[0, 0, 0])
    qddot = model.forward_dynamics(q, qdot, tau)
    print(f"qddot (with more external forces): {qddot}")

    # Remove all external forces
    model.external_force_set.reset()
    qddot = model.forward_dynamics(q, qdot, tau)
    print(f"qddot (after removing external forces): {qddot}")


if __name__ == "__main__":
    main()
