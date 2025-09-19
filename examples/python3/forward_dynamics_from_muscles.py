from pathlib import Path

import numpy as np
import biorbd

#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen state (position (q), velocity (qdot))
#     3. Compute the generalized acceleration (qddot) assuming a set
#        of muscle control (activations)
#     4. Print them to the console
#     5. Repeat the same exercise but accounting for muscle activations dynamics,
#        using muscle excitations as control and adding the muscle activation as state
# Please note that this example will work only with the Eigen backend
#


def main():
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Biorbd(f"{current_file_dir}/../arm26.bioMod")
    muscles = model.muscles
    nmus = len(muscles)

    # Choose a state (position/velocity) to compute dynamics from
    activations = [0.5] * nmus
    q = [0.1] * model.nb_q
    qdot = [0.1] * model.nb_qdot

    # # Warning, failing to provide the "q" and "qdot" will compute the joint torques at previous pose, which can be useful
    # # if the kinematics was manually updated, but is a bug otherwise
    # tau_uninitialized_pose = model.muscles.joint_torque(activations=activations)

    # # Compute the same activations but at the chosen pose and velocity. There are two ways of doing so depending on the
    # # level of control you want on the internal updating scheme of the model
    # # 1. The "low-level" way, where you manually update the kinematics
    # muscles.activations = activations
    # muscles.update_geometry(q=q, qdot=qdot)
    # tau = muscles.joint_torque()  # No need to send activations, q or qdot as they were already sent
    # 2. The "high-level" way, where the kinematics is updated internally
    tau = muscles.joint_torque(activations=activations, q=q, qdot=qdot)

    # To get generalized accelerations, we simply need to inject the joint torque in the forward dynamics
    qddot = model.forward_dynamics(q, qdot, tau)
    print(f"Generalized accelerations (qddot) are: {qddot}")

    # As an extra, let's print the individual muscle forces (note that we don't update activations, q or qdot here,
    # as they were already sent to compute tau, but we could if needed)
    print(f"Muscle forces at q={q}: {model.muscles.forces()}")
    # By comparison, this updates the kinematics to q = [0, 0]
    print(f"Muscle forces at q=[0.0, 0.0]: {model.muscles.forces(q=[0, 0])}")

    # qddot needs to be integrated twice to compute the new state (q, qdot).
    # Choosing a new control (muscle activation), this small exercise should be repeated to move forward in time.

    # EXCITATION-DRIVEN DYNAMICS

    # Choose a state (position/velocity/activation) to compute dynamics from
    q = [0.1] * model.nb_q
    qdot = [0.1] * model.nb_qdot
    initial_activations = [0.5] * nmus
    excitations = np.arange(1, nmus + 1) / nmus  # Varying excitations from 0.1 to 1.0

    # Set all muscles to their current activations and excitations

    # Compute the derivatives of muscle activations, which should be used in
    # a forward integration to compute the time evolution of the muscle activation
    # If this the next two lines are manually done, they can be omitted from the next function call
    # muscles.activations = initial_activations
    # muscles.excitations = excitations
    activations_dot = muscles.activations_dot(excitations=excitations, activations=initial_activations)

    # Print them to the console
    print(f"Derivative of activations: {activations_dot}")


if __name__ == "__main__":
    main()
