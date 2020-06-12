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


# Load a predefined model
model = biorbd.Model("../pyomecaman.bioMod")
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
