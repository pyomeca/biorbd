from pathlib import Path

import numpy as np
import biorbd

try:
    import bioviz

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


def main(show: bool = True):
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Biorbd(f"{current_file_dir}/../pyomecaman.bioMod")
    n_frames = 20

    # Generate clapping gesture data
    qinit = [0, 0, -0.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0]
    qmid = [0, 0, -0.3, 0.5, 1.15, -0.5, 1.15, 0, 0, 0, 0, 0, 0]
    qfinal = [0, 0, -0.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0]
    target_q = np.concatenate((np.linspace(qinit, qmid, n_frames).T, np.linspace(qmid, qfinal, n_frames).T), axis=1)
    markers = []
    for q in target_q.T:
        markers.append(np.array([mark.position for mark in model.markers(q)]).T)

    # If ones was using c3d data opened using ezc3d (instead of generated data as above)
    # import ezc3d
    # c3d = ezc3d.c3d(data_filename)
    # markers = c3d['data']['points'][:3, :, :]  # XYZ1 x markers x time_frame

    # Perform the kalman filter for each frame (remember, due to initialization, first frame is much longer than the rest)
    kalman = biorbd.ExtendedKalmanFilterMarkers(model, frequency=100)
    q_recons = np.ndarray(target_q.shape)
    for i, (q_i, _, _) in enumerate(kalman.reconstruct_frames(markers)):
        q_recons[:, i] = q_i

        # Print the kinematics to the console
        print(f"Frame {i}\nExpected Q = {target_q[:, i]}\nCurrent Q = {q_i}\n")

    # # If ones wants to manually iterate through the frames they can call reconstruct_frame instead passing the ith markers
    # # This is useful in online applications
    # for i, marker in enumerate(markers):
    #     q_i, _, _ = kalman.reconstruct_frame(marker)
    #     q_recons[:, i] = q_i

    # Animate the results if biorbd viz is installed
    if show and biorbd_viz_found:
        b = bioviz.Viz(loaded_model=model)
        b.load_movement(q_recons)
        b.exec()


if __name__ == "__main__":
    main()
