import numpy as np
import biorbd

#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen position (Q), velocity (Qdot)
#     3. Compute the generalized acceleration (Qddot) assuming a set
#        muscle activations (joint torque from muscle)
#     4. Print them to the console
#
# Please note that this example will work only with the Eigen backend
#


# Load a predefined model
model = biorbd.Model("../arm26.bioMod")
nq = model.nbQ()
nqdot = model.nbQdot()
nmus = model.nbMuscles()

# Choose a position/velocity to compute dynamics from
Q = np.zeros((nq,))
Qdot = np.zeros((nqdot,))

# Set all muscles to half of their half-maximal activation
emg = model.stateSet()
for e in emg:
    e.setActivation(0.5)  # Set muscles activations

# Proceed with the computation of joint torque from the muscles
Tau = model.muscularJointTorque(emg, Q, Qdot)

# Compute the generalized accelerations using the Tau from muscles.
# Please note that in forward dynamics setting, it is usually advised to
# additionnal residual torques. You would add them here to Tau.
Qddot = model.ForwardDynamics(Q, Qdot, Tau)

# Print them to the console
print(Qddot.to_array())

# As an extra, let's print the individual muscle forces
muscle_forces = model.muscleForces(emg, Q, Qdot)
print(muscle_forces.to_array())
