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


# Load a predefined model
model = biorbd.Model("../arm26.bioMod")
nq = model.nbQ()
nqdot = model.nbQdot()
nqddot = model.nbQddot()
ntau = model.nbGeneralizedTorque()
n_frames = 3

# Choose a position/velocity/torque to compute muscle activations from.
# If only one frame the Vector are not mandatory and the Static Optimization function can be called
# directly with numpy arrays
Q = biorbd.VecBiorbdGeneralizedCoordinates()
Qdot = biorbd.VecBiorbdGeneralizedVelocity()
Qddot = biorbd.VecBiorbdGeneralizedAcceleration()
Tau = biorbd.VecBiorbdGeneralizedTorque()
for i in range(n_frames):
    Q.append(np.zeros((nq,)))
    Qdot.append(np.zeros((nqdot,)))
    Qddot.append(np.zeros((nqddot,)))
    Tau.append(model.InverseDynamics(Q[i], Qdot[i], Qddot[i]))

# Proceed with the static optimization
optim = biorbd.StaticOptimization(model, Q, Qdot, Tau)
optim.run()
muscleActivationsPerFrame = optim.finalSolution()

# Print them to the console
for activations in muscleActivationsPerFrame:
    print(activations.to_array())
