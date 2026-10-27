from dataclasses import dataclass

from scipy import optimize
import numpy as np

from .utils import get_range_q

try:
    import proxsuite

    has_proxsuite = True
except ImportError:
    proxsuite = None
    has_proxsuite = False


@dataclass
class DifferentialInverseKinematicsResult:
    """
    Per-frame result returned by the ProxQP differential IK solver.

    The named fields describe the ProxQP solve more explicitly than scipy's
    OptimizeResult names. The fun, nfev and njev properties are kept so that
    InverseKinematics.sol() can summarize both scipy and ProxQP results.
    """

    residual_vector: np.ndarray
    marker_function_evaluations: int
    jacobian_evaluations: int
    message: str
    status: int
    success: bool

    @property
    def fun(self):
        return self.residual_vector

    @property
    def nfev(self):
        return self.marker_function_evaluations

    @property
    def njev(self):
        return self.jacobian_evaluations


@dataclass
class MarkerLoopClosureConstraint:
    """
    Linearized loop-closure constraint between two technical markers.

    The constraint enforces that both markers occupy the same global position.
    At each differential IK linearization, it contributes:

    ``(J_predecessor - J_successor) dq = -(p_predecessor - p_successor)``.
    """

    predecessor_marker: str | int
    successor_marker: str | int
    axes: tuple[int, ...] = (0, 1, 2)

    @staticmethod
    def _marker_index(marker_names: list[str], marker: str | int):
        if isinstance(marker, int):
            return marker
        try:
            return marker_names.index(marker)
        except ValueError:
            raise ValueError(f"{marker} is not in the biorbd model") from None

    def residual_and_jacobian(self, inverse_kinematics, q: np.ndarray):
        predecessor_index = self._marker_index(
            inverse_kinematics.marker_names, self.predecessor_marker
        )
        successor_index = self._marker_index(
            inverse_kinematics.marker_names, self.successor_marker
        )
        axes = np.asarray(self.axes, dtype=int)

        markers = np.array(inverse_kinematics.biorbd_model.technicalMarkers(q))
        jacobians = np.array(
            inverse_kinematics.biorbd_model.technicalMarkersJacobian(q)
        )

        residual = (
            markers[predecessor_index].to_array() - markers[successor_index].to_array()
        )[axes]
        jacobian = (
            jacobians[predecessor_index].to_array()
            - jacobians[successor_index].to_array()
        )[axes, :]
        return residual, jacobian


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
            self.biorbd_model.markerNames()[i].to_string()
            for i in range(len(self.biorbd_model.markerNames()))
        ]
        self.nb_markers = self.biorbd_model.nbMarkers()

        if isinstance(marker_data, np.ndarray):
            if (
                marker_data.ndim >= 2
                and marker_data.shape[0] <= 3
                and marker_data.shape[1] == self.nb_markers
            ):
                self.xp_markers = marker_data
                self.nb_frames = marker_data.shape[2]
            else:
                raise ValueError(
                    f"The standard dimension of the NumPy array should be (nb_dim, nb_marker, nb_frame)"
                )
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
            self.indices_to_remove.append(
                list(np.unique(np.isnan(self.xp_markers[:, :, j]).nonzero()[1]))
            )
            self.indices_to_keep.append(
                list(np.unique(np.isfinite(self.xp_markers[:, :, j]).nonzero()[1]))
            )

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
            raise ValueError(
                'This method is not implemented please use "trf", "lm" or "only_lm" as argument'
            )

        for f in range(self.nb_frames):
            if f % 100 == 0:
                print(f"Frame {f}/{self.nb_frames}")
            if initial_method != "lm":
                x0 = (
                    np.array(
                        [
                            (bounds_inf + bounds_sup) / 2
                            for bounds_inf, bounds_sup in zip(
                                initial_bounds[0], initial_bounds[1]
                            )
                        ]
                    )
                    if f == 0
                    else self.q[:, f - 1]
                )
            else:
                x0 = np.ones(self.nb_q) * 0.0001 if f == 0 else self.q[:, f - 1]

            sol = optimize.least_squares(
                fun=lambda q, marker_real, indices_to_keep: self._marker_diff(
                    np.array(self.biorbd_model.technicalMarkers(q))[indices_to_keep],
                    marker_real,
                ),
                args=(
                    self.xp_markers[:, :, f][:, self.indices_to_keep[f]],
                    self.indices_to_keep[f],
                ),
                bounds=initial_bounds if f == 0 else bounds,
                jac=lambda q, jacobian_matrix, indices_to_keep: self._marker_jacobian(
                    np.array(self.biorbd_model.technicalMarkersJacobian(q))[
                        indices_to_keep
                    ]
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
            indices_to_keep_xyz = [
                h * self.nb_dim + k
                for h in self.indices_to_keep[f]
                for k in range(self.nb_dim)
            ]
            indices_to_remove_xyz = [
                h * self.nb_dim + k
                for h in self.indices_to_remove[f]
                for k in range(self.nb_dim)
            ]

            # After we get the indices, we put the solution on the residuals
            residuals_xyz[:, f][indices_to_keep_xyz] = self.list_sol[f].fun

            # The markers that we removed have 0 as position on x, y and z but this is not true.
            # So we replace it by nan value.
            residuals_xyz[:, f][indices_to_remove_xyz] = np.nan
            residuals[:, f] = np.linalg.norm(
                np.reshape(residuals_xyz[:, f], [self.nb_markers, self.nb_dim]), axis=1
            )

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


class DifferentialInverseKinematics(InverseKinematics):
    """
    Differential inverse kinematics solver using ProxQP.

    This class solves a sequence of linearized marker-tracking QPs. It handles
    marker tracking, generalized-coordinate bounds, and optional linearized
    marker loop-closure constraints.

    Compared to the scipy least-squares backend, this solver exposes a QP step
    at each linearization. That makes it easier to add linear constraints and
    can be efficient on warm-started frame sequences. The tradeoff is that
    it is a local differential method: difficult frames may require several
    linearizations, and the result depends on the initial guess more directly
    than a full nonlinear least-squares solve.

    This implementation keeps the frame and linearization loops in Python. That
    keeps the prototype easy to inspect and test, but performance-critical uses
    or large trials should eventually move this loop to a compiled C++ class.
    """

    def _initial_guess(self, initial_q: np.ndarray | None):
        if initial_q is not None:
            q = np.asarray(initial_q, dtype=float)
            if q.shape != (self.nb_q,):
                raise ValueError(f"initial_q should have shape ({self.nb_q},)")
            return q.copy()

        q_min, q_max = self.bounds
        q = np.zeros(self.nb_q)
        finite_bounds = np.isfinite(q_min) & np.isfinite(q_max)
        q[finite_bounds] = (q_min[finite_bounds] + q_max[finite_bounds]) / 2
        lower_only = np.isfinite(q_min) & ~np.isfinite(q_max)
        q[lower_only] = q_min[lower_only]
        upper_only = ~np.isfinite(q_min) & np.isfinite(q_max)
        q[upper_only] = q_max[upper_only]
        return q

    @staticmethod
    def _proxqp_bounds(bounds: tuple[np.ndarray, np.ndarray], q: np.ndarray):
        lower, upper = bounds
        lower = lower - q
        upper = upper - q
        lower = np.where(np.isfinite(lower), lower, -1e20)
        upper = np.where(np.isfinite(upper), upper, 1e20)
        return lower, upper

    def _solve_qp(
        self,
        hessian: np.ndarray,
        gradient: np.ndarray,
        lower_bounds: np.ndarray,
        upper_bounds: np.ndarray,
        tolerance: float,
        equality_matrix: np.ndarray | None = None,
        equality_vector: np.ndarray | None = None,
    ):
        nb_equalities = 0 if equality_matrix is None else equality_matrix.shape[0]
        qp = proxsuite.proxqp.dense.QP(self.nb_q, nb_equalities, self.nb_q)
        qp.settings.eps_abs = tolerance
        qp.settings.eps_rel = tolerance
        qp.settings.verbose = False
        equality_matrix = (
            np.empty((0, self.nb_q)) if equality_matrix is None else equality_matrix
        )
        equality_vector = np.empty((0,)) if equality_vector is None else equality_vector
        qp.init(
            hessian,
            gradient,
            equality_matrix,
            equality_vector,
            np.eye(self.nb_q),
            lower_bounds,
            upper_bounds,
        )
        qp.solve()
        return np.array(qp.results.x).reshape(self.nb_q)

    def _constraints_residual_and_jacobian(
        self, q: np.ndarray, constraints: list[MarkerLoopClosureConstraint]
    ):
        if not constraints:
            return np.empty((0,)), np.empty((0, self.nb_q))

        residuals = []
        jacobians = []
        for constraint in constraints:
            residual, jacobian = constraint.residual_and_jacobian(self, q)
            residuals.append(residual)
            jacobians.append(jacobian)

        return np.concatenate(residuals), np.vstack(jacobians)

    def solve(
        self,
        initial_q: np.ndarray | None = None,
        marker_weights: np.ndarray | None = None,
        max_iterations: int = 10,
        tolerance: float = 1e-8,
        regularization: float = 1e-8,
        constraints: list[MarkerLoopClosureConstraint] | None = None,
        constraint_tolerance: float | None = None,
        verbose: bool = True,
    ):
        """
        Solve inverse kinematics by repeated differential QPs.

        Parameters
        ----------
        initial_q: np.ndarray | None
            Initial generalized coordinates for the first frame. If None, the
            midpoint of finite model bounds is used when possible.
        marker_weights: np.ndarray | None
            One scalar weight per model marker. Missing markers are ignored.
        max_iterations: int
            Maximum number of QP linearization steps per frame.
        tolerance: float
            Stopping tolerance on marker residual norm and ProxQP accuracy.
        regularization: float
            Positive diagonal regularization added to the QP Hessian.
        constraints: list[MarkerLoopClosureConstraint] | None
            Linearized equality constraints to enforce at each QP step.
        constraint_tolerance: float | None
            Stopping tolerance on the nonlinear constraint residual norm. If
            None, the marker tolerance is used.
        verbose: bool
            If True, prints frame progress every 100 frames.

        Returns
        -------
        q : np.ndarray
            Generalized coordinates with shape (nb_q, nb_frames).
        """
        if not has_proxsuite:
            raise RuntimeError(
                "DifferentialInverseKinematics requires the optional dependency 'proxsuite'."
            )
        if max_iterations < 1:
            raise ValueError("max_iterations must be at least 1")
        if regularization < 0:
            raise ValueError("regularization must be non-negative")
        if constraint_tolerance is None:
            constraint_tolerance = tolerance
        constraints = [] if constraints is None else constraints

        if marker_weights is None:
            marker_weights = np.ones(self.nb_markers)
        marker_weights = np.asarray(marker_weights, dtype=float)
        if marker_weights.shape != (self.nb_markers,):
            raise ValueError(f"marker_weights should have shape ({self.nb_markers},)")

        bounds = self.bounds
        q = self._initial_guess(initial_q)

        for f in range(self.nb_frames):
            if verbose and f % 100 == 0:
                print(f"Frame {f}/{self.nb_frames}")

            if f != 0:
                q = self.q[:, f - 1].copy()

            residual = np.array([])
            success = False
            iterations = 0
            for iterations in range(1, max_iterations + 1):
                indices_to_keep = self.indices_to_keep[f]
                if not indices_to_keep:
                    break

                markers_real = self.xp_markers[:, :, f][:, indices_to_keep]
                residual = self._marker_diff(
                    np.array(self.biorbd_model.technicalMarkers(q))[indices_to_keep],
                    markers_real,
                )
                if np.linalg.norm(residual) <= tolerance:
                    constraint_residual, _ = self._constraints_residual_and_jacobian(
                        q, constraints
                    )
                    if np.linalg.norm(constraint_residual) <= constraint_tolerance:
                        success = True
                        break

                jacobian = self._marker_jacobian(
                    np.array(self.biorbd_model.technicalMarkersJacobian(q))[
                        indices_to_keep
                    ]
                )
                weights = np.repeat(marker_weights[indices_to_keep], 3)
                weighted_jacobian = jacobian * weights[:, np.newaxis]
                hessian = jacobian.T @ weighted_jacobian
                hessian += np.eye(self.nb_q) * regularization
                gradient = jacobian.T @ (weights * residual)

                lower_bounds, upper_bounds = self._proxqp_bounds(bounds, q)
                (
                    constraint_residual,
                    constraint_jacobian,
                ) = self._constraints_residual_and_jacobian(q, constraints)
                dq = self._solve_qp(
                    hessian,
                    gradient,
                    lower_bounds,
                    upper_bounds,
                    tolerance,
                    constraint_jacobian,
                    -constraint_residual,
                )
                q = q + dq

                if np.linalg.norm(dq) <= tolerance:
                    residual = self._marker_diff(
                        np.array(self.biorbd_model.technicalMarkers(q))[
                            indices_to_keep
                        ],
                        markers_real,
                    )
                    constraint_residual, _ = self._constraints_residual_and_jacobian(
                        q, constraints
                    )
                    success = (
                        np.linalg.norm(residual) <= tolerance
                        and np.linalg.norm(constraint_residual) <= constraint_tolerance
                    )
                    break

            if self.indices_to_keep[f]:
                markers_real = self.xp_markers[:, :, f][:, self.indices_to_keep[f]]
                residual = self._marker_diff(
                    np.array(self.biorbd_model.technicalMarkers(q))[
                        self.indices_to_keep[f]
                    ],
                    markers_real,
                )
                constraint_residual, _ = self._constraints_residual_and_jacobian(
                    q, constraints
                )
                success = success or (
                    np.linalg.norm(residual) <= tolerance
                    and np.linalg.norm(constraint_residual) <= constraint_tolerance
                )

            self.q[:, f] = q
            message = (
                "Converged"
                if success
                else "Maximum number of QP linearizations reached"
            )
            self.list_sol.append(
                DifferentialInverseKinematicsResult(
                    residual_vector=residual,
                    marker_function_evaluations=iterations,
                    jacobian_evaluations=iterations,
                    message=message,
                    status=1 if success else 0,
                    success=success,
                )
            )

        return self.q


InverseKinematicsProxQP = DifferentialInverseKinematics
