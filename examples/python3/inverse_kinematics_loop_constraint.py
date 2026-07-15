from pathlib import Path
from tempfile import NamedTemporaryFile
import contextlib
import io
import time

import numpy as np
from scipy import optimize

import biorbd


def _loop_model_with_markers():
    """
    Create a temporary copy of the loop-constrained model with tracking markers.

    The original test model declares the loop constraint, but has no markers.
    This example adds two hidden closure markers and six visible tracking
    markers so inverse kinematics can be benchmarked.
    """
    repo_root = Path(__file__).resolve().parents[2]
    model_path = repo_root / "test" / "models" / "loopConstrainedModel.bioMod"
    model_text = model_path.read_text()
    marker_text = """
marker loop_pre
  parent Segment5
  position 0 0 0
endmarker

marker loop_succ
  parent Segment3
  position 0 0 0
endmarker

marker track_s2a
  parent Segment2
  position 0.1 0.2 -0.4
endmarker

marker track_s2b
  parent Segment2
  position -0.1 0.1 -1.0
endmarker

marker track_s3
  parent Segment3
  position 0.1 -0.1 -0.8
endmarker

marker track_s4a
  parent Segment4
  position -0.1 0.2 -0.4
endmarker

marker track_s4b
  parent Segment4
  position 0.1 0.1 -1.0
endmarker

marker track_s5
  parent Segment5
  position -0.1 -0.1 -0.8
endmarker

"""
    model_text = model_text.replace(
        "\nloopconstraint\n", marker_text + "loopconstraint\n", 1
    )
    model_file = NamedTemporaryFile("w", suffix=".bioMod", delete=False)
    model_file.write(model_text)
    model_file.close()
    return Path(model_file.name)


def _markers_to_array(model, q):
    return np.array([marker.to_array() for marker in model.technicalMarkers(q)]).T


def _loop_gap(model, q):
    markers = _markers_to_array(model, q).T
    return markers[0] - markers[1]


def _project_to_loop(model, q_desired, initial_q):
    def residual(q):
        return np.concatenate((1000 * _loop_gap(model, q), 0.1 * (q - q_desired)))

    sol = optimize.least_squares(
        residual,
        initial_q,
        max_nfev=200,
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    return sol.x


def _generate_closed_loop_data(model, n_frames):
    q = np.zeros(model.nbQ())
    q_ref = np.ndarray((model.nbQ(), n_frames))
    phase = np.linspace(0, 2 * np.pi, n_frames)

    for frame, phase_i in enumerate(phase):
        q_desired = 0.15 * np.sin(phase_i + np.arange(model.nbQ()) * 0.3)
        q = _project_to_loop(model, q_desired, q)
        q_ref[:, frame] = q

    markers = np.ndarray((3, model.nbMarkers(), n_frames))
    for frame in range(n_frames):
        markers[:, :, frame] = _markers_to_array(model, q_ref[:, frame])

    # The closure markers are model-only constraints, not measured markers.
    markers[:, 0:2, :] = np.nan
    return q_ref, markers


def _summarize(model, name, q, ik, elapsed):
    sol = ik.sol()
    return _summarize_q(model, name, q, elapsed, sol["residuals"], sol["nfev"])


def _summarize_q(model, name, q, elapsed, marker_residuals, iterations):
    loop_gaps = np.array(
        [np.linalg.norm(_loop_gap(model, q[:, f])) for f in range(q.shape[1])]
    )
    return {
        "method": name,
        "s_per_frame": elapsed / q.shape[1],
        "mean_marker_residual_m": np.nanmean(marker_residuals),
        "mean_loop_gap_m": loop_gaps.mean(),
        "max_loop_gap_m": loop_gaps.max(),
        "mean_iterations": np.mean(iterations),
    }


def _run_inverse_kinematics(model, markers, method):
    ik = biorbd.InverseKinematics(model, markers)
    tic = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        q = ik.solve(method=method)
    return q, ik, time.perf_counter() - tic


def _visible_marker_residuals(model, q, markers_real, indices_to_keep):
    markers_model = _markers_to_array(model, q)
    residuals = markers_model[:, indices_to_keep] - markers_real[:, indices_to_keep]
    return residuals.T.reshape(-1)


def _run_trf_loop(model, markers, initial_q, loop_weight=100):
    q_out = np.ndarray((model.nbQ(), markers.shape[2]))
    marker_residuals = np.ndarray((model.nbMarkers(), markers.shape[2])) * np.nan
    iterations = []
    q = initial_q.copy()
    tic = time.perf_counter()

    for frame in range(markers.shape[2]):
        indices_to_keep = list(
            np.unique(np.isfinite(markers[:, :, frame]).nonzero()[1])
        )

        def residual(q_tp):
            return np.concatenate(
                (
                    _visible_marker_residuals(
                        model, q_tp, markers[:, :, frame], indices_to_keep
                    ),
                    loop_weight * _loop_gap(model, q_tp),
                )
            )

        sol = optimize.least_squares(
            residual,
            q,
            method="trf",
            xtol=1e-10,
            ftol=1e-10,
            gtol=1e-10,
            max_nfev=200,
        )
        q = sol.x
        q_out[:, frame] = q
        iterations.append(sol.nfev)

        marker_residual_vector = _visible_marker_residuals(
            model, q, markers[:, :, frame], indices_to_keep
        )
        marker_residuals[indices_to_keep, frame] = np.linalg.norm(
            marker_residual_vector.reshape(-1, 3), axis=1
        )

    elapsed = time.perf_counter() - tic
    return q_out, marker_residuals, iterations, elapsed


def _run_differential_inverse_kinematics(model, markers, initial_q, constraints):
    ik = biorbd.DifferentialInverseKinematics(model, markers)
    tic = time.perf_counter()
    q = ik.solve(
        initial_q=initial_q,
        max_iterations=20,
        tolerance=1e-8,
        constraints=constraints,
        constraint_tolerance=1e-8,
        verbose=False,
    )
    return q, ik, time.perf_counter() - tic


def main(n_frames=20):
    model_path = _loop_model_with_markers()
    model = biorbd.Model(str(model_path))
    q_ref, markers = _generate_closed_loop_data(model, n_frames)

    rows = []
    for method in ("only_lm", "lm", "trf"):
        q, ik, elapsed = _run_inverse_kinematics(model, markers, method)
        rows.append(_summarize(model, method, q, ik, elapsed))

    q, marker_residuals, iterations, elapsed = _run_trf_loop(
        model, markers, initial_q=q_ref[:, 0]
    )
    rows.append(
        _summarize_q(model, "trf_loop", q, elapsed, marker_residuals, iterations)
    )

    q, ik, elapsed = _run_differential_inverse_kinematics(
        model, markers, initial_q=q_ref[:, 0], constraints=None
    )
    rows.append(_summarize(model, "proxqp_dik", q, ik, elapsed))

    loop_constraint = biorbd.MarkerLoopClosureConstraint("loop_pre", "loop_succ")
    q, ik, elapsed = _run_differential_inverse_kinematics(
        model, markers, initial_q=q_ref[:, 0], constraints=[loop_constraint]
    )
    rows.append(_summarize(model, "proxqp_dik_loop", q, ik, elapsed))

    print(
        "method,s_per_frame,mean_marker_residual_m,"
        "mean_loop_gap_m,max_loop_gap_m,mean_iterations"
    )
    for row in rows:
        print(
            f"{row['method']},{row['s_per_frame']:.8f},"
            f"{row['mean_marker_residual_m']:.12g},"
            f"{row['mean_loop_gap_m']:.12g},"
            f"{row['max_loop_gap_m']:.12g},"
            f"{row['mean_iterations']:.3g}"
        )

    return rows


if __name__ == "__main__":
    main()
