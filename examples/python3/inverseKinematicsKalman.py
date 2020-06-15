import numpy as np
import biorbd

#
# This examples shows how to
#     1. Load a model
#     2. Generate data (should be acquired via real data)
#     3. Create a Kalman filter
#     4. Apply the Kalman filter (inverse kinematics)
#     5. Plot the kinematics (Q), velocity (Qdot) and acceleration (Qddot)
#
# Please note that this example will work only with the Eigen backend.
# Please also note that kalman will be VERY slow if compiled in debug
#


# Load a predefined model
model = biorbd.Model("../pyomecaman.bioMod")
nq = model.nbQ()
# nqdot = model.nbQdot()
# nqddot = model.nbQddot()
n_frames = 3

# Generate random data (3 frames)
targetQ = np.random.random((nq,))
targetMarkers = model.markers(targetQ)

markersOverFrames = []
for i in range(n_frames):
    markersOverFrames.append(targetMarkers)

# Create a Kalman filter
freq = 100  # 100 Hz
params = biorbd.KalmanParam(freq)
kalman = biorbd.KalmanReconsMarkers(model, params)

# Perform the kalman filter for each frame (the first frame is much longer than the next)
print(f"Target Q = {targetQ}")
Q = biorbd.GeneralizedCoordinates(model)
Qdot = biorbd.GeneralizedVelocity(model)
Qddot = biorbd.GeneralizedAcceleration(model)
for targetMarkers in markersOverFrames:
    kalman.reconstructFrame(model, targetMarkers, Q, Qdot, Qddot)

    # Print the kinematics to the console
    print(Q.to_array())
