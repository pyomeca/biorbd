from scipy import optimize
import numpy as np

from . import biorbd
from .utils import get_range_q


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
    markers_in_c3d[:, index_in_c3d >= 0, :] = data[:3, index_in_c3d[index_in_c3d >= 0], frames] / 1000  # To meter

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


class InverseKinematics:
    """
    The class for generate inverse kinematics from c3d files

    Attributes:
    ----------
    biorbd_model: biorbd.Model
        The biorbd model loaded
    marker_names: list[str]
        The list of markers' name
    nb_markers: int
        The number of markers in the model
    xp_markers: np.array
        The position of the markers from the c3d
    nb_frames: int
        The number of frame in the c3d
    nb_q: int
        The number of dof in the model
    q: np.array
        generalized coordinates
    bounds: tuple(np.ndarray, np.ndarray)
        The min and max ranges of the model Q
    indices_to_remove: list(list(int))
        The list of markers index  which have a nan value in xp_markers
    indices_to_keep: list(list(int))
        The list of markers index  which have a number value in xp_markers
    list_sol: list(scipy.OptimizeResult)
        The list of results of least_square function
    output: dict()
        The output of the solution:
            residuals_xyz: np.ndarray
                The array of the final return of _marker_diff function.
                The final difference between markers position in the model and in the c3d.
            residuals: np.ndarray
                The array of the norm of residuals_xyz position for each markers in each frame.
            nfev: np.ndarray
                The array of the number of iteration of the _marker_diff function for each frame.
            njev: np.ndarray
                The array of the number of iteration of the _marker_jac function for each frame.
            max_marker: list(str)
                The list of markers that have the highest residual.
                So the markers that have the biggest difference between the model and the c3d for each frame.
            message: list(str)
                The list of the verbal description of the termination reason of the least_square function for each frame.
            status: list(int)
                The reason for algorithm termination for each frame
                -1 : improper input parameters status returned from MINPACK.
                0 : the maximum number of function evaluations is exceeded.
                1 : gtol termination condition is satisfied.
                2 : ftol termination condition is satisfied.
                3 : xtol termination condition is satisfied.
                4 : Both ftol and xtol termination conditions are satisfied.
            success: list(bool)
                The list of success for each frame. True if one of the convergence criteria is satisfied (status > 0).
    nb_dim: int
        The number of dimension of the model

    Methods
    -------
    _get_nan_index(self)
        Find, for each frame, the index of the markers which has a nan value
    _marker_diff(markers_model: np.ndarray, markers_real: np.ndarray)
        Compute the difference between the marker position in the model and the position in the data.
    _marker_jacobian(self, jacobian_matrix)
        Generate the Jacobian matrix for each frame.
    optimize(self, n_frame: int, method: str, bounds: tuple() = None)
        Uses least_square function to minimize the difference between markers' positions of model and c3d.
    solve(self, method: str = "lm")
        Solve the inverse kinematics by using least_square method from scipy.
    sol(self)
        Create and return a dict which contains the output each optimization.

    """

    def __init__(
        self,
        model,
        marker_data: np.ndarray,
    ):
        """
        Parameters
        ----------
        model: biorbd.Model
            The biorbd model loaded with biorbd Eigen backend
        marker_data: np.ndarray
            The position of the markers from the c3d of shape (nb_dim, nb_marker, nb_frame),
            nb_marker should be equal to the number of markers in the model, unit should be in meters.
        """
        self.biorbd_model = model
        self.marker_names = [
            self.biorbd_model.markerNames()[i].to_string() for i in range(len(self.biorbd_model.markerNames()))
        ]
        self.nb_markers = self.biorbd_model.nbMarkers()

        if isinstance(marker_data, np.ndarray):
            if marker_data.ndim >= 2 and marker_data.shape[0] <= 3 and marker_data.shape[1] == self.nb_markers:
                self.xp_markers = marker_data
                self.nb_frames = marker_data.shape[2]
            else:
                raise ValueError(f"The standard dimension of the NumPy array should be (nb_dim, nb_marker, nb_frame)")
        else:
            raise ValueError("The standard inputs is a numpy.ndarray")

        self.nb_q = self.biorbd_model.nbQ()
        self.q = np.zeros((self.nb_q, self.nb_frames))

        self.bounds = get_range_q(self.biorbd_model)

        self.indices_to_remove = []
        self.indices_to_keep = []
        self._get_nan_index()

        self.list_sol = []

        self.output = dict()
        self.nb_dim = self.xp_markers.shape[0]

    def _get_nan_index(self):
        """
        Find, for each frame, the index of the markers which has a nan value
        """
        for j in range(self.nb_frames):
            self.indices_to_remove.append(list(np.unique(np.isnan(self.xp_markers[:, :, j]).nonzero()[1])))
            self.indices_to_keep.append(list(np.unique(np.isfinite(self.xp_markers[:, :, j]).nonzero()[1])))

    @staticmethod
    def _marker_diff(markers_model: np.ndarray, markers_real: np.ndarray):
        """
        Compute the difference between the marker position in the model and the position in the data

        Parameters:
        -----------
        markers_model: np.ndarray
            he position of the markers from the model during a certain frame
        markers_real: np.ndarray
            The position of the markers from the c3d during a certain frame

        Return:
        ------
            The difference vector between markers' position in the model and in the c3d
        """
        nb_marker = len(markers_real[0])

        vect_pos_markers = np.zeros(3 * nb_marker)

        for m, value in enumerate(markers_model):
            vect_pos_markers[m * 3 : (m + 1) * 3] = value.to_array()

        return vect_pos_markers - np.reshape(markers_real.T, (3 * nb_marker,))

    def _marker_jacobian(self, jacobian_matrix: np.ndarray):
        """
        Generate the Jacobian matrix for each frame.

        Parameters:
        -----------
        jacobian_matrix: np.ndarray
            The Jacobian matrix of the model

        Return:
        ------
            The Jacobian matrix with right dimension
        """
        nb_markers = len(jacobian_matrix)
        jacobian = np.zeros((3 * nb_markers, self.nb_q))

        for m, value in enumerate(jacobian_matrix):
            jacobian[m * 3 : (m + 1) * 3, :] = value.to_array()

        return jacobian

    def solve(self, method: str = "lm"):
        """
        Solve the inverse kinematics by using least_square method from scipy

        Parameters:
        ----------
        method: str
            The method used by least_square to optimize the difference.

            If method = 'lm', the 'trf' method will be used for the first frame, in order to respect the bounds of the model.
            Then, the 'lm' method will be used for the following frames.
            If method = 'trf', the 'trf' method will be used for all the frames.
            If method = 'only_lm', the 'lm' method will be used for all the frames.

            In least_square:
                -‘trf’ : Trust Region Reflective algorithm, particularly suitable for large sparse problems
                        with bounds.
                        Generally robust method.
                -‘lm’ : Levenberg-Marquardt algorithm as implemented in MINPACK.
                        Doesn’t handle bounds and sparse Jacobians.
                        Usually the most efficient method for small unconstrained problems.

        Returns
        ----------
        q : np.array
            generalized coordinates
        """
        initial_bounds = (-np.inf, np.inf) if method == "only_lm" else self.bounds
        initial_method = "lm" if method == "only_lm" else "trf"

        bounds = self.bounds if method == "trf" else (-np.inf, np.inf)
        method = "lm" if method == "only_lm" else method

        if method != "lm" and method != "trf" and method != "only_lm":
            raise ValueError('This method is not implemented please use "trf", "lm" or "only_lm" as argument')

        for f in range(self.nb_frames):
            if initial_method != "lm":
                x0 = (
                    np.array(
                        [
                            (bounds_inf + bounds_sup) / 2
                            for bounds_inf, bounds_sup in zip(initial_bounds[0], initial_bounds[1])
                        ]
                    )
                    if f == 0
                    else self.q[:, f - 1]
                )
            else:
                x0 = np.ones(self.nb_q) * 0.0001 if f == 0 else self.q[:, f - 1]

            sol = optimize.least_squares(
                fun=lambda q, marker_real, indices_to_keep: self._marker_diff(
                    np.array(self.biorbd_model.technicalMarkers(q))[indices_to_keep], marker_real
                ),
                args=(self.xp_markers[:, :, f][:, self.indices_to_keep[f]], self.indices_to_keep[f]),
                bounds=initial_bounds if f == 0 else bounds,
                jac=lambda q, jacobian_matrix, indices_to_keep: self._marker_jacobian(
                    np.array(self.biorbd_model.technicalMarkersJacobian(q))[indices_to_keep]
                ),
                x0=x0,
                method=initial_method if f == 0 else method,
                xtol=1e-6,
                tr_options=dict(disp=False),
            )
            self.q[:, f] = sol.x
            self.list_sol.append(sol)
        return self.q

    def sol(self):
        """
        Create and return a dict that contains the output of each optimization.

        Return
        ------
        self.output: dict()
            The output of least_square function, such as number of iteration per frames,
            and the marker with highest residual
        """
        residuals_xyz = np.zeros((self.nb_markers * self.nb_dim, self.nb_frames))
        residuals = np.zeros((self.nb_markers, self.nb_frames))
        nfev = [sol.nfev for sol in self.list_sol]
        njev = [sol.njev for sol in self.list_sol]

        for f in range(self.nb_frames):
            #  residuals_xyz must contains position for each markers on axis x, y and z
            #  (or less depending on number of dimensions)
            #  So, if we simply used indices_to_keep or to_remove, it doesn't work because it contains the markers'
            #  indices [markers 0, markers 4, ...].
            #  We have to create another list which contains indices for x, y and z
            # [indices_to_keep or remove for markers 0 on x, indices_to_keep or remove for markers 0 on y, ...]
            # And that correspond to [0, 1, 2, 4, 5, 6, ...] in our example.
            indices_to_keep_xyz = [h * self.nb_dim + k for h in self.indices_to_keep[f] for k in range(self.nb_dim)]
            indices_to_remove_xyz = [h * self.nb_dim + k for h in self.indices_to_remove[f] for k in range(self.nb_dim)]

            # After we get the indices, we put the solution on the residuals
            residuals_xyz[:, f][indices_to_keep_xyz] = self.list_sol[f].fun

            # The markers that we removed have 0 as position on x, y and z but this is not true.
            # So we replace it by nan value.
            residuals_xyz[:, f][indices_to_remove_xyz] = np.nan
            residuals[:, f] = np.linalg.norm(np.reshape(residuals_xyz[:, f], [self.nb_markers, self.nb_dim]), axis=1)

        self.output = dict(
            residuals=residuals,
            residuals_xyz=residuals_xyz,
            nfev=nfev,
            njev=njev,
            max_marker=[self.marker_names[i] for i in np.argmax(residuals, axis=0)],
            message=[sol.message for sol in self.list_sol],
            status=[sol.status for sol in self.list_sol],
            success=[sol.success for sol in self.list_sol],
        )

        return self.output
