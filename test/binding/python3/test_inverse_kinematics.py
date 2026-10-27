"""
Test for file IO
"""

import pytest
import numpy as np
from pathlib import Path
from scipy import optimize

brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd: {e}")
    pass

try:
    import biorbd_casadi

    brbd_to_test.append(biorbd_casadi)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd_casadi: {e}")
    pass

if not brbd_to_test:
    raise RuntimeError("No biorbd version could be imported")


def _loop_model_with_markers(tmp_path):
    model_path = tmp_path / "loopConstrainedModelWithMarkers.bioMod"
    source_model_path = (
        Path(__file__).parents[2] / "models" / "loopConstrainedModel.bioMod"
    )
    with open(source_model_path) as model_file:
        model_text = model_file.read()

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
    model_path.write_text(model_text)
    return model_path


def _technical_markers_to_array(model, q):
    return np.array([marker.to_array() for marker in model.technicalMarkers(q)]).T


def _loop_gap(model, q):
    markers = _technical_markers_to_array(model, q).T
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


@pytest.mark.parametrize("brbd", brbd_to_test)
@pytest.mark.parametrize("method", ["only_lm", "lm", "trf"])
def test_solve(brbd, method):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse kinematics for biorbd_casadi")

    biorbd_model = brbd.Model("../../models/pyomecaman.bioMod")

    # Remove the dampings in this test
    joint_dampings = [0, 0, 0]
    biorbd_model.segment(0).setJointDampings(joint_dampings)

    qinit = np.array([0.1, 0.1, -0.3, 0.35, 1.15, -0.35, 1.15, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    markers = np.ndarray((3, biorbd_model.nbMarkers(), 1))
    markers[:, :, 0] = np.array([mark.to_array() for mark in biorbd_model.markers(qinit)]).T

    ik = biorbd.InverseKinematics(biorbd_model, markers)
    ik_q = ik.solve(method=method)

    if method == "only_lm":
        np.testing.assert_almost_equal(
            np.squeeze(ik_q.T),
            qinit,
        )
    elif method == "trf" or method == "lm":
        np.testing.assert_almost_equal(np.squeeze(np.round(ik_q, 1).T), qinit, decimal=1)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_differential_inverse_kinematics_requires_proxsuite(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse kinematics for biorbd_casadi")
    if brbd.has_proxsuite:
        pytest.skip("This test covers the missing optional dependency path")

    biorbd_model = brbd.Model("../../models/pyomecaman.bioMod")
    markers = np.zeros((3, biorbd_model.nbMarkers(), 1))
    ik = brbd.DifferentialInverseKinematics(biorbd_model, markers)

    with pytest.raises(RuntimeError, match="proxsuite"):
        ik.solve(verbose=False)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_differential_inverse_kinematics_solve(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse kinematics for biorbd_casadi")
    if not brbd.has_proxsuite:
        pytest.skip("proxsuite is not installed")

    biorbd_model = brbd.Model("../../models/pyomecaman.bioMod")

    # Remove the dampings in this test
    joint_dampings = [0, 0, 0]
    biorbd_model.segment(0).setJointDampings(joint_dampings)

    qinit = np.array(
        [0.1, 0.1, -0.3, 0.35, 1.15, -0.35, 1.15, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    )

    markers = np.ndarray((3, biorbd_model.nbMarkers(), 1))
    markers[:, :, 0] = np.array(
        [mark.to_array() for mark in biorbd_model.markers(qinit)]
    ).T

    ik = brbd.DifferentialInverseKinematics(biorbd_model, markers)
    ik_q = ik.solve(
        initial_q=np.zeros(biorbd_model.nbQ()),
        max_iterations=10,
        tolerance=1e-8,
        verbose=False,
    )

    np.testing.assert_almost_equal(np.squeeze(np.round(ik_q, 1).T), qinit, decimal=1)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_differential_inverse_kinematics_ignores_missing_markers(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse kinematics for biorbd_casadi")
    if not brbd.has_proxsuite:
        pytest.skip("proxsuite is not installed")

    biorbd_model = brbd.Model("../../models/pyomecaman.bioMod")
    qinit = np.array(
        [0.1, 0.1, -0.3, 0.35, 1.15, -0.35, 1.15, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    )

    markers = np.ndarray((3, biorbd_model.nbMarkers(), 1))
    markers[:, :, 0] = np.array(
        [mark.to_array() for mark in biorbd_model.markers(qinit)]
    ).T
    markers[:, 0, 0] = np.nan

    ik = brbd.DifferentialInverseKinematics(biorbd_model, markers)
    ik.solve(initial_q=qinit, max_iterations=2, tolerance=1e-8, verbose=False)
    residuals = ik.sol()["residuals"]

    assert np.isnan(residuals[0, 0])
    assert np.nanmax(residuals[:, 0]) < 1e-8


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_differential_inverse_kinematics_with_marker_loop_constraint(brbd, tmp_path):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse kinematics for biorbd_casadi")
    if not brbd.has_proxsuite:
        pytest.skip("proxsuite is not installed")

    biorbd_model = brbd.Model(str(_loop_model_with_markers(tmp_path)))
    q_closed = _project_to_loop(
        biorbd_model,
        0.1 * np.sin(np.arange(biorbd_model.nbQ()) * 0.3),
        np.zeros(biorbd_model.nbQ()),
    )

    markers = np.ndarray((3, biorbd_model.nbMarkers(), 1))
    markers[:, :, 0] = _technical_markers_to_array(biorbd_model, q_closed)
    markers[:, 0:2, 0] = np.nan

    ik = brbd.DifferentialInverseKinematics(biorbd_model, markers)
    q = ik.solve(
        initial_q=q_closed,
        max_iterations=5,
        tolerance=1e-8,
        constraints=[brbd.MarkerLoopClosureConstraint("loop_pre", "loop_succ")],
        constraint_tolerance=1e-8,
        verbose=False,
    )

    np.testing.assert_almost_equal(
        np.linalg.norm(_loop_gap(biorbd_model, q[:, 0])), 0
    )


if __name__ == "__main__":
    for brbd in brbd_to_test:
        for method in ["only_lm", "lm", "trf"]:
            test_solve(brbd, method)
