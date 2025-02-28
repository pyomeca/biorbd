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


# ACTIVATION-DRIVEN DYNAMICS

def main():
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Model(f"{current_file_dir}/../arm26.bioMod")
    nq = model.nbQ()
    nqdot = model.nbQdot()
    nmus = model.nbMuscles()

    # Choose a state (position/velocity) to compute dynamics from
    q = np.zeros((nq,))
    qdot = np.zeros((nqdot,))

    # Set an arbitrary control to all muscles (half of their maximal activation)
    muscles = model.stateSet()
    for muscle in muscles:
        muscle.setActivation(0.5)  # Set muscles activations

    # Now one can compute the muscle forces
    muscle_forces = model.muscleForces(muscles, q, qdot)

    # Proceed with the computation of joint torque from the muscles
    tau = model.muscularJointTorque(muscles, q, qdot)

    # Compute the generalized accelerations using the tau from muscles.
    # Please note that in forward dynamics setting, it is usually advised to add
    # additional residual torques. You would add them here to tau.
    qddot = model.ForwardDynamics(q, qdot, tau)

    # Print them to the console
    print(qddot.to_array())

    # As an extra, let's print the individual muscle forces
    print(muscle_forces.to_array())

    # qddot needs to be integrated twice to compute the new state (q, qdot).
    # Choosing a new control (muscle activation), this small exercise should be repeated to move forward in time.

    # EXCITATION-DRIVEN DYNAMICS

    # Choose a state (position/velocity/activation) to compute dynamics from
    q = np.zeros((nq,))
    qdot = np.zeros((nqdot,))
    act = np.zeros((nmus,))

    # Set all muscles to their current activation and to an arbitrary excitation of 0.5
    muscles = model.stateSet()
    for k, muscle in enumerate(muscles):
        muscle.setActivation(act[k])  # Set muscles activations
        muscle.setExcitation(0.5)  # Set muscles activations

    # Compute the derivatives of muscle activations, which should be used in
    # a forward integration to compute the time evolution of the muscle activation
    actdot = model.activationDot(muscles)

    # Print them to the console
    print(actdot.to_array())

    # Proceed with the computation of joint torque from the muscles
    tau = model.muscularJointTorque(muscles, q, qdot)

    # Compute the generalized accelerations using the tau from muscles.
    qddot = model.ForwardDynamics(q, qdot, tau)

    # qddot needs to be integrated twice and actdot integrated once to compute the new state (q, qdot, act).
    # Choosing a new control (muscle excitations), this small exercise should be repeated to move forward in time.


if __name__ == "__main__":
    main()
