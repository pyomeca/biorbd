import numpy as np
import biorbd
try:
    import BiorbdViz
    biorbd_viz_found = True
except ModuleNotFoundError:
    biorbd_viz_found = False


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
nb_mus = model.nbMuscles()
n_frames = 20

# Generate clapping gesture data
qinit = np.array([0, 0, -.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0])
qmid = np.array([0, 0, -.3, 0.5, 1.15, -0.5, 1.15, 0, 0, 0, 0, 0, 0])
qfinal = np.array([0, 0, -.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0])
target_q = np.concatenate((np.linspace(qinit, qmid, n_frames).T, np.linspace(qmid, qfinal, n_frames).T), axis=1)
markers = np.ndarray((3, model.nbMarkers(), 2*n_frames))
for i, q in enumerate(target_q.T):
    markers[:, :, i] = np.array([mark.to_array() for mark in model.markers(q)]).T

# If ones was using c3d data opened using ezc3d 
# import ezc3d
# c3d = ezc3d.c3d(data_filename)
# markers = c3d['data']['points'][:3, :, :]  # XYZ1 x markers x time_frame

# Dispatch markers in biorbd structure so EKF can use it
markersOverFrames = []
for i in range(markers.shape[2]):
    markersOverFrames.append([biorbd.NodeSegment(m) for m in markers[:, :, i].T])

# Create a Kalman filter structure
freq = 100  # Hz
params = biorbd.KalmanParam(freq)
kalman = biorbd.KalmanReconsMarkers(model, params)

# Perform the kalman filter for each frame (the first frame is much longer than the next)
Q = biorbd.GeneralizedCoordinates(model)
Qdot = biorbd.GeneralizedVelocity(model)
Qddot = biorbd.GeneralizedAcceleration(model)
q_recons = np.ndarray((model.nbQ(), len(markersOverFrames)))
for i, targetMarkers in enumerate(markersOverFrames):
    kalman.reconstructFrame(model, targetMarkers, Q, Qdot, Qddot)
    q_recons[:, i] = Q.to_array()

    # Print the kinematics to the console
    print(f"Frame {i}\nExpected Q = {target_q[:, i]}\nCurrent Q = {q_recons[:, i]}\n")

# Animate the results if biorbd viz is installed
if biorbd_viz_found:
    b = BiorbdViz.BiorbdViz(loaded_model=model)
    b.load_movement(q_recons)
    b.exec()

