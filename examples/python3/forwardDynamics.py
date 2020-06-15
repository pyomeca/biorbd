import numpy as np
import biorbd

#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen position (Q), velocity (Qdot)
#     3. Compute the generalized acceleration (Qddot) assuming a set of generalized forces (forward dynamics)
#     4. Print them to the console
#
# Please note that this example will work only with the Eigen backend
#


# Load a predefined model
model = biorbd.Model("../pyomecaman.bioMod")
nq = model.nbQ()
nqdot = model.nbQdot()
ntau = model.nbGeneralizedTorque()

# Choose a position/velocity/torque to compute dynamics from
Q = np.zeros((nq,))
Qdot = np.zeros((nqdot,))
Tau = np.zeros((ntau,))

# Proceed with the forward dynamics
Qddot = model.ForwardDynamics(Q, Qdot, Tau)

# Print them to the console
print(Qddot.to_array())
