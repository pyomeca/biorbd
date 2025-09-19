from pathlib import Path

import numpy as np
import biorbd

#
# This examples shows how to
#     1. Load a model with muscles
#     2. Position the model at a chosen position (Q) and velocity (Qdot)
#     3. Define a target generalized forces (Tau)
#     4. Compute the muscle activations that reproduce this Tau (Static optimization)
#     5. Print them to the console
#
# Please note that this example will work only with the Eigen backend
#


def main():
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Biorbd(f"{current_file_dir}/../arm26.bioMod")
    n_frames = 3

    # Choose a position/velocity/torque to compute muscle activations from.
    # If only one frame the Vector are not mandatory and the Static Optimization function can be called
    # directly with numpy arrays
    q = []
    qdot = []
    qddot = []
    tau = []
    for i in range(n_frames):
        # The q would typically come from an inverse kinematics analysis
        q.append([0] * model.nb_q)
        # The qdot and qddot would typically come from a numerical differentiation of the kinematics
        qdot.append([0] * model.nb_qdot)
        qddot.append([0] * model.nb_qddot)

        # The tau would typically come from an inverse dynamics analysis as follow
        tau.append(model.inverse_dynamics(q[i], qdot[i], qddot[i]))

    # Proceed with the static optimization. When perform is called, all the frames are processed at once, even though
    # it is a loop. That is so the initial guess is dependent of the previous frame. So the first "frame" of the loop is
    # very long (as it computes everythin). Then, the following frames are very fast (as it only returns the precomputed
    # results)
    optim = biorbd.StaticOptimization(model)
    muscle_activations = []
    for value in optim.perform_frames(q, qdot, tau):
        muscle_activations.append(value)

    # Print them to the console
    for i, activations in enumerate(muscle_activations):
        print(f"Frame {i}: {activations}")


if __name__ == "__main__":
    main()
