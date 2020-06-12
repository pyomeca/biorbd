import numpy as np
import biorbd

#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen position (Q)
#     3. Compute the position of the skin markers at that position (Forward kinematics)
#     4. Print them to the console
#
# Please note that this example will work only with the Eigen backend
#


# Load a predefined model
model = biorbd.Model("../pyomecaman.bioMod")
nq = model.nbQ()

# Choose a position to get the markers from
Q = np.zeros((nq,))

# Proceed with the forward kinematics
markers = model.markers(Q)

# Print them to the console
for marker in markers:
    print(marker.to_array())
