import numpy as np

from . import biorbd
from .inverse_kinematics import (
    DifferentialInverseKinematics,
    DifferentialInverseKinematicsResult,
    InverseKinematics,
    InverseKinematicsProxQP,
    MarkerLoopClosureConstraint,
    has_proxsuite,
)


def marker_index(model, marker_name: str) -> int:
    """
    Return the index in the model of the desired marker.
    A ValueError is raised if the marker is not in the model

    Parameters
    ----------
    model: biorbd.Model
        The biorbd model
    marker_name: str
        The name of the marker to get the index from
    Returns
    -------
    The index of the marker.
    """

    try:
        return [n.to_string() for n in model.markerNames()].index(marker_name)
    except ValueError:
        raise ValueError(f"{marker_name} is not in the biorbd model")

def contact_index(model, contact_name: str) -> int:
    """
    Return the index in the model of the desired contact point.
    A ValueError is raised if the marker is not in the model

    Parameters
    ----------
    model: biorbd.Model
        The biorbd model
    contact_name: str
        The name of the contact to get the index from
    Returns
    -------
    The index of the contact.
    """

    try:
        return [n.to_string() for n in model.contactNames()].index(contact_name)
    except ValueError:
        raise ValueError(f"{contact_name} is not in the biorbd model")

def segment_index(model, segment_name):
    """
    Return the index in the model of the desired segment.
    A ValueError is raised if the segment is not in the model

    Parameters
    ----------
    model: biorbd.Model
        The biorbd model
    segment_name: str
        The name of the segment to get the index from
    Returns
    -------
    The index of the segment.
    """

    try:
        return [model.segment(i).name().to_string() for i in range(model.nbSegment())].index(segment_name)
    except ValueError:
        raise ValueError(f"{segment_name} is not in the biorbd model")


def markers_to_array(model, q: np.ndarray) -> np.ndarray:
    """
    Get all markers position from a position q in the format (3 x NMarker x NTime).
    This function probably only works with the Eigen backend

    Parameters
    ----------
    model: biorbd.Model
        The biorbd model
    q: np.ndarray
        The matrix of generalized coordinate in the format (NDof x NTime)

    Returns
    -------
    The markers position in the format (3 x NMarker x NTime)
    """

    markers = np.ndarray((3, model.nbMarkers(), q.shape[1]))
    for i, q_tp in enumerate(q.T):
        markers[:, :, i] = np.array([mark.to_array() for mark in model.markers(q_tp)]).T
    return markers


def extended_kalman_filter(
    model: biorbd.Model, trial: str, frames: slice = slice(None)
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Reconstruct the kinematics of the specified trial assuming a biorbd model is loaded using an Extended Kalman filter

    Parameters
    ----------
    model
        The model to use to reconstruct the kinematics
    trial
        The path to the c3d file of the trial to reconstruct the kinematics from
    frames
        The frames that should be used in the reconstruction

    Returns
    -------
    First element is the time vector, the next three are the q, qdot and qddot computed from the EKF.
    These three matrices are of size nq x ntimes
    """
    import ezc3d

    marker_names = tuple(n.to_string() for n in model.technicalMarkerNames())
    frames = slice(None) if frames is None else frames

    c3d = ezc3d.c3d(trial, extract_forceplat_data=True)
    labels = c3d["parameters"]["POINT"]["LABELS"]["value"]
    data = c3d["data"]["points"]
    n_frames = data[:, :, frames].shape[2]

    index_in_c3d = np.array(tuple(labels.index(name) if name in labels else -1 for name in marker_names))
    markers_in_c3d = np.ndarray((3, len(index_in_c3d), n_frames)) * np.nan
    markers_in_c3d[:, index_in_c3d[index_in_c3d >= 0], :] = data[:3, index_in_c3d[index_in_c3d >= 0], frames] / 1000  # To meter

    # Create a Kalman filter structure
    freq = c3d["parameters"]["POINT"]["RATE"]["value"][0]
    params = biorbd.KalmanParam(freq)
    kalman = biorbd.KalmanReconsMarkers(model, params)

    # Perform the kalman filter for each frame (the first frame is much longer than the next)
    q = biorbd.GeneralizedCoordinates(model)
    qdot = biorbd.GeneralizedVelocity(model)
    qddot = biorbd.GeneralizedAcceleration(model)
    frame_rate = c3d["header"]["points"]["frame_rate"]
    first_frame = c3d["header"]["points"]["first_frame"] if frames.start is None else frames.start
    last_frame = c3d["header"]["points"]["last_frame"] if frames.stop is None else frames.stop
    t = np.linspace(first_frame / frame_rate, last_frame / frame_rate, n_frames)
    q_out = np.ndarray((model.nbQ(), n_frames))
    qdot_out = np.ndarray((model.nbQdot(), n_frames))
    qddot_out = np.ndarray((model.nbQddot(), n_frames))
    for i in range(n_frames):
        kalman.reconstructFrame(model, np.reshape(markers_in_c3d[:, :, i].T, -1), q, qdot, qddot)
        q_out[:, i] = q.to_array()
        qdot_out[:, i] = qdot.to_array()
        qddot_out[:, i] = qddot.to_array()

    return t, q_out, qdot_out, qddot_out
