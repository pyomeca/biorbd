from scipy import optimize

from .utils import *


def marker_index(model, marker_name):
    try:
        return [n.to_string() for n in model.markerNames()].index(marker_name)
    except ValueError:
        raise ValueError(f"{marker_name} is not in the biorbd model")


def segment_index(model, segment_name):
    try:
        return [model.segment(i).name().to_string() for i in range(model.nbSegment())].index(segment_name)
    except ValueError:
        raise ValueError(f"{segment_name} is not in the biorbd model")


class InverseKinematics:
    """
    The class for generate inverse kinematics from c3d files

    Attributes:
    ----------
    biorbd_model_path: str
        The biorbd model path
    biorbd_model: biorbd.Model
        The biorbd model loaded
    marker_names: list[str]
        The list of markers' name
    xp_markers: np.array
        The position of the markers from the c3d
    nb_q: int
        The number of dof in the model
    nb_frames: int
        The number of frame in the c3d
    nb_markers: int
        The number of markers in the model
    q: np.array
        generalized coordinates
    bounds: tuple(np.ndarray, np.ndarray)
        The min and max ranges of the model Q
    idx_to_remove: list(int)
        The list of markers index  which have a nan value in xp_markers
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

    Methods
    -------
    _get_marker_trajectories(self)
        get markers trajectories
    _get_idx_to_remove(self)
        Find, for each frame, the index of the markers which has a nan value
    _marker_diff(self, q: np.ndarray, xp_markers: np.ndarray)
        Compute the difference between the marker position in the model and the position in the data.
    _marker_jac(self, q: np.ndarray, xp_markers: np.ndarray)
        Generate the Jacobian matrix for each frame.
    optimize(self, n_frame: int, method: str, bounds: tuple() = None)
        Uses least_square function to minimize the difference between markers' positions of model and c3d.
    solve(self, method: str = "lm", full: bool = False)
        Solve the inverse kinematics by using least square methode from scipy.
    sol(self)
        Create and return a dict which contains the output each optimization.

    """

    def __init__(
        self,
        model,
        marker_data: np.array,
    ):
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
            raise ValueError("The standard inputs are str, an ezc3d.c3d, or a numpy.ndarray")

        self.nb_q = self.biorbd_model.nbQ()
        self.indices_to_remove = [[]] * self.nb_frames  # todo: check the independence
        self._get_nan_index()
        self.q = np.zeros((self.nb_q, self.nb_frames))
        self.bounds = get_range_q(self.biorbd_model)

        self.list_sol = []

        self.output = dict()
        self.nb_dim = self.xp_markers.shape[0]

    def _get_nan_index(self):
        """
        Find, for each frame, the index of the markers which has a nan value
        """
        for j in range(self.nb_frames):
            self.indices_to_remove[j] = list(np.unique(np.isnan(self.xp_markers[:, :, j]).nonzero()[1]))

    def _marker_diff(self, q: np.ndarray, xp_markers: np.ndarray, idx_to_remove):
        """
        Compute the difference between the marker position in the model and the position in the data

        Parameters:
        -----------
        q: np.ndarray
            The q values
        xp_markers: np.ndarray
            The position of the markers from the c3d during a certain frame

        Return:
        ------
            The difference vector between markers' position in the model and in the c3d
        """
        mat_pos_markers = self.biorbd_model.technicalMarkers(q)
        # remove index of model markers
        mat_pos_markers = np.delete(mat_pos_markers, idx_to_remove, 0)
        # remove nan in experimental data
        xp_markers = np.delete(xp_markers, idx_to_remove, 1)

        vect_pos_markers = np.zeros(3 * self.nb_markers)

        for m, value in enumerate(mat_pos_markers):  # todo : check if reshape
            vect_pos_markers[m * 3 : (m + 1) * 3] = value.to_array()

        return vect_pos_markers - np.reshape(xp_markers.T, (self.nb_markers * 3,))

    def _marker_jacobian(self, q: np.ndarray, xp_markers: np.ndarray, idx_to_remove):
        """
        Generate the Jacobian matrix for each frame.

        Parameters:
        -----------
        q: np.ndarray
            The q values
        xp_markers: np.ndarray
            The position of the markers from the c3d during a certain frame

        Return:
        ------
            The Jacobian matrix
        """
        jacobian_matrix = self.biorbd_model.technicalMarkersJacobian(q)
        jacobian_matrix = np.delete(jacobian_matrix, idx_to_remove, 0)
        nb_markers = len(jacobian_matrix)
        jac = np.zeros((3 * nb_markers, self.nb_q))

        for m, value in enumerate(jacobian_matrix):
            jac[m * 3 : (m + 1) * 3, :] = value.to_array()

        return jac

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
                    np.random.uniform(initial_bounds[0], initial_bounds[1], self.nb_q) * 0.1
                    if f == 0
                    else self.q[:, f - 1]
                )
            else:
                x0 = np.random.random(self.nb_q) * 0.1 if f == 0 else self.q[:, f - 1]

            sol = optimize.least_squares(
                fun=self._marker_diff,
                args=(self.xp_markers[:, :, f], self.indices_to_remove[f]),
                bounds=initial_bounds if f == 0 else bounds,
                jac=self._marker_jacobian,
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
        residuals_xyz = np.ndarray((self.nb_markers * self.nb_dim, self.nb_frames))
        residuals = np.ndarray((self.nb_markers, self.nb_frames))
        nfev = [sol.nfev for sol in self.list_sol]
        njev = [sol.njev for sol in self.list_sol]

        for i in range(self.nb_frames):
            residuals_xyz[:, i] = self.list_sol[i].fun
            residuals[:, i] = np.linalg.norm(np.reshape(residuals_xyz[:, i], [self.nb_markers, self.nb_dim]), axis=1)

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
