import re

brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except:
    pass
try:
    import biorbd_casadi
    from casadi import MX

    brbd_to_test.append(biorbd_casadi)
except:
    pass
import numpy as np
import pytest

if not brbd_to_test:
    raise ImportError("No biorbd version could be imported")


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_model(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    # Generic properties
    assert model.name == "pyomecaman"
    assert model.path == "../../models/pyomecaman.bioMod"

    # Gravity
    np.testing.assert_almost_equal(model.gravity, [0, 0, -9.81])
    model.gravity = [1, 2, 3]
    np.testing.assert_almost_equal(model.gravity, [1, 2, 3])


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_segments(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    # Test accessors
    assert len(model.segments) == len(model.internal.segments())
    assert model.segments[0].name == "Pelvis"
    assert model.segments["Pelvis"].name == "Pelvis"

    # Test specific segment
    segment = model.segments[0]

    # Name
    assert segment.name == "Pelvis"

    # Translation and rotation sequences
    assert segment.translations == "yz"
    assert segment.rotations == "x"

    # Mass
    assert segment.mass == 9.03529
    segment.mass = 100
    assert segment.mass == 100

    # Center of mass
    np.testing.assert_almost_equal(segment.center_of_mass, [0, 0, 0.0885])
    segment.center_of_mass = [1, 2, 3]
    np.testing.assert_almost_equal(segment.center_of_mass, [1, 2, 3])

    # Inertia
    np.testing.assert_almost_equal(segment.inertia, [[0.04664, 0.0, 0.0], [0.0, 0.07178, 0.0], [0.0, 0.0, 0.06989]])
    segment.inertia = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    np.testing.assert_almost_equal(segment.inertia, [[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    # Get the markers of the segment
    assert len(segment.markers) == 6


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_markers(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    assert len(model.markers) == len(model.internal.markers())

    # Test accessors
    assert model.markers[0].name == "pelv1"
    assert model.markers["pelv1"].name == "pelv1"

    # Test a specific marker
    marker = model.markers[0]

    # Name
    assert marker.name == "pelv1"

    # Parent segment
    assert marker.segment.name == "Pelvis"

    # Position
    np.testing.assert_almost_equal(marker.local, [-0.1038, 0.0821, 0.0])
    marker.local = [1, 2, 3]
    np.testing.assert_almost_equal(marker.local, [1, 2, 3])

    # X, Y, Z
    assert marker.x == 1
    marker.x = 4
    assert marker.x == 4

    assert marker.y == 2
    marker.y = 5
    assert marker.y == 5

    assert marker.z == 3
    marker.z = 6
    assert marker.z == 6

    # Is technical or anatomical
    assert marker.is_anatomical is False
    assert marker.is_technical is False

    # Perform FK to get world position
    q = [0.1] * model.nb_q
    # First try without updating kinematics
    markers = model.markers
    np.testing.assert_almost_equal(markers[0].world, [4, 5, 6])

    # Then with updating kinematics
    markers = markers(q)
    np.testing.assert_almost_equal(markers[0].world, [4.0, 4.47602033, 6.56919207])

    # Then test that the update_kinematics is still applied
    markers = model.markers
    np.testing.assert_almost_equal(markers[0].world, [4.0, 4.47602033, 6.56919207])

    # Test the jacobian of first marker at previous set q a set q
    jacobian_at_q = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, -6.46919207, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 4.37602033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    jacobian_at_2q = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, -6.87374612, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 3.7083169, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    np.testing.assert_almost_equal(marker.jacobian(), jacobian_at_q)
    np.testing.assert_almost_equal(marker.jacobian(q=np.array(q) * 2), jacobian_at_2q)

    # Test the all markers jacobian
    jacobian = markers.jacobian()
    assert len(jacobian) == len(model.markers)
    np.testing.assert_almost_equal(jacobian[0], jacobian_at_2q)
    np.testing.assert_almost_equal(markers.jacobian(q=q)[0], jacobian_at_q)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_frames(brbd):
    model = brbd.Biorbd("../../models/arm26.bioMod")

    # Test accessors
    assert len(model.segment_frames) == len(model.segments)
    assert model.segment_frames[2].name == model.segment_frames["r_humerus_rotation1"].name
    frame = model.segments[2].frame

    q = [0.1, 0.1]

    # Compute the reference frames of each segment at that position
    # For clarity sake, just print the first segment, in normal use, one would probably want to use all segments
    np.testing.assert_almost_equal(
        frame.local,
        [
            [0.99750108, 0.03902081, -0.05889802, 0.0],
            [-0.03895296, 0.9992384, 0.0023, 0.0],
            [0.05894291, 0.0, 0.99826136, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )
    np.testing.assert_almost_equal(frame.world, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    np.testing.assert_almost_equal(
        frame(q),
        [
            [0.99641331, -0.06075807, -0.05889802, -0.017545],
            [0.06099902, 0.99813518, 0.0023, -0.007],
            [0.05864844, -0.00588447, 0.99826136, 0.17],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )
    np.testing.assert_almost_equal(
        frame.world,
        [
            [0.99641331, -0.06075807, -0.05889802, -0.017545],
            [0.06099902, 0.99813518, 0.0023, -0.007],
            [0.05864844, -0.00588447, 0.99826136, 0.17],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )

    # We can extract some useful information from the frame
    np.testing.assert_almost_equal(
        frame.local_rotation,
        [[0.99750108, 0.03902081, -0.05889802], [-0.03895296, 0.9992384, 0.0023], [0.05894291, 0.0, 0.99826136]],
    )
    np.testing.assert_almost_equal(frame.local_rotation_as_euler("xyz"), [-0.002304, -0.05893213, -0.03909863])
    np.testing.assert_almost_equal(frame.local_translation, [0.0, 0.0, 0.0])
    np.testing.assert_almost_equal(
        frame.world_rotation,
        [
            [0.99641331, -0.06075807, -0.05889802],
            [0.06099902, 0.99813518, 0.0023],
            [0.05864844, -0.00588447, 0.99826136],
        ],
    )
    np.testing.assert_almost_equal(frame.world_rotation_as_euler("xyz"), [-0.002304, -0.05893213, 0.06090137])
    np.testing.assert_almost_equal(frame.world_translation, [-0.017545, -0.007, 0.17])


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_dynamics(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    assert model.nb_q == model.internal.nbQ()
    assert model.nb_qdot == model.internal.nbQdot()
    assert model.nb_qddot == model.internal.nbQddot()
    assert model.nb_tau == model.internal.nbGeneralizedTorque()
    assert model.dof_names == [dof.to_string() for dof in model.internal.nameDof()]

    if brbd.currentLinearAlgebraBackend() == 1:
        q_sym = MX.sym("q", (model.nb_q,))
        qdot_sym = MX.sym("qdot", (model.nb_qdot,))
        qddot_sym = MX.sym("qddot", (model.nb_qdot,))
        tau_sym = MX.sym("tau", (model.nb_tau,))

        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", model.forward_dynamics, q_sym, qdot_sym, tau_sym)
        forward_dynamics_internal = brbd.to_casadi_func(
            "ForwardDynamicsInternal", model.internal.ForwardDynamics, q_sym, qdot_sym, tau_sym
        )

        inverse_dynamics = brbd.to_casadi_func("InverseDynamics", model.inverse_dynamics, q_sym, qdot_sym, qddot_sym)
        inverse_dynamics_internal = brbd.to_casadi_func(
            "InverseDynamicsInternal", model.internal.InverseDynamics, q_sym, qdot_sym, qddot_sym
        )

    else:
        forward_dynamics = model.forward_dynamics
        forward_dynamics_internal = model.internal.ForwardDynamics

        inverse_dynamics = model.inverse_dynamics
        inverse_dynamics_internal = model.internal.InverseDynamics

    q = np.arange(model.nb_q)
    qdot = np.arange(model.nb_qdot)
    qddot = np.arange(model.nb_qdot)
    tau = np.arange(model.nb_tau)

    np.testing.assert_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=True), forward_dynamics_internal(q, qdot, tau).to_array()
    )
    np.testing.assert_almost_equal(
        inverse_dynamics(q, qdot, qddot), inverse_dynamics_internal(q, qdot, qddot).to_array()
    )


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_external_forces(brbd):
    # Load a predefined model
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    # Segment on which the external force will be applied
    segment = model.segments[0]

    # Testing failing cases
    with pytest.raises(ValueError, match=re.escape("The input vector must be of size 3 (force) or 6 (spatial vector)")):
        model.external_force_set.add(segment_name=segment.name, force=[0, 0], point_of_application=[0, 0, 0])
        model.external_force_set.add(segment_name=segment.name, force=[0, 0, 0, 0], point_of_application=[0, 0, 0])
    with pytest.raises(
        ValueError,
        match="The point of application must be provided when adding a force in the world frame",
    ):
        model.external_force_set.add(segment_name=segment.name, force=[0, 0, 9.81 * segment.mass])
    with pytest.raises(
        ValueError,
        match=re.escape(
            "Adding a force in local frame is not implemented (and probably not what you want). "
            "You probably want to add a spatial vector in the local frame instead or a force in the world frame."
        ),
    ):
        model.external_force_set.add(
            segment_name=segment.name,
            force=[0, 0, 9.81 * segment.mass],
            frame_of_reference=brbd.ExternalForceSet.Frame.LOCAL,
            point_of_application=[0, 0, 0],
        )
    with pytest.raises(
        ValueError,
        match="The point of application must be provided when adding a spatial vector in the local frame",
    ):
        model.external_force_set.add(
            segment_name=segment.name,
            force=[0, 0, 0, 0, 0, 9.81 * segment.mass],
            frame_of_reference=brbd.ExternalForceSet.Frame.LOCAL,
        )

    if brbd.currentLinearAlgebraBackend() == 1:
        q_sym = MX.sym("q", (model.nb_q,))
        qdot_sym = MX.sym("qdot", (model.nb_qdot,))
        tau_sym = MX.sym("tau", (model.nb_tau,))

        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", model.forward_dynamics, q_sym, qdot_sym, tau_sym)
    else:
        forward_dynamics = model.forward_dynamics

    # Computing forward dynamics as reference
    q = [0] * model.nb_q
    qdot = [0] * model.nb_qdot
    tau = [0] * model.nb_tau
    ref_qddot = forward_dynamics(q=q, qdot=qdot, tau=tau, ignore_contacts=True, ignore_external_forces=True)
    np.testing.assert_array_almost_equal(
        ref_qddot,
        [0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    )

    # Test adding one external force to the model
    model.external_force_set.add(
        segment_name=segment.name, force=[0, 0, 9.81 * segment.mass], point_of_application=[0, 0, 0]
    )
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=False),
        [
            -0.195144,
            -0.141795,
            5.03609,
            15.715842,
            -8.472491,
            -15.715842,
            -8.472491,
            -18.12955,
            27.010416,
            -13.916956,
            -18.12955,
            27.010416,
            -13.916956,
        ],
    )
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=True),
        [
            0.697101,
            -7.923693,
            2.846458,
            2.374355,
            -3.754325,
            -2.374355,
            -3.754325,
            -7.568292,
            6.753389,
            -18.305875,
            -7.568292,
            6.753389,
            -18.305875,
        ],
    )

    # Add an extra external force to the model that pushes upward even more
    model.external_force_set.add(segment_name=segment.name, force=[0, 0, 1], point_of_application=[0, 0, 0])
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=False),
        [
            -0.185022,
            -0.139734,
            5.062955,
            15.709644,
            -8.503352,
            -15.709644,
            -8.503352,
            -18.190191,
            27.044657,
            -13.917421,
            -18.190191,
            27.044657,
            -13.917421,
        ],
    )
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=True),
        [
            0.704966,
            -7.902412,
            2.878572,
            2.401142,
            -3.796682,
            -2.401142,
            -3.796682,
            -7.653678,
            6.829581,
            -18.512403,
            -7.653678,
            6.829581,
            -18.512403,
        ],
    )

    # Remove all external forces
    model.external_force_set.reset()
    np.testing.assert_array_almost_equal(forward_dynamics(q, qdot, tau, ignore_contacts=True), ref_qddot)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_kalman_filter(brbd):
    # Load a predefined model
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")
    n_frames = 20

    # Generate clapping gesture data
    qinit = [0, 0, -0.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0]
    qmid = [0, 0, -0.3, 0.5, 1.15, -0.5, 1.15, 0, 0, 0, 0, 0, 0]
    qfinal = [0, 0, -0.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0]
    target_q = np.concatenate((np.linspace(qinit, qmid, n_frames).T, np.linspace(qmid, qfinal, n_frames).T), axis=1)
    markers = []
    for q in target_q.T:
        markers.append(np.array([mark.world for mark in model.markers(q)]).T)

    # Perform the kalman filter for each frame (remember, due to initialization, first frame is much longer than the rest)
    kalman = brbd.ExtendedKalmanFilterMarkers(model, frequency=100)
    q_recons = np.ndarray(target_q.shape)
    for i, (q_i, _, _) in enumerate(kalman.reconstruct_frames(markers)):
        q_recons[:, i] = q_i
    np.testing.assert_almost_equal(q_recons[:, -1], target_q[:, -1], decimal=6)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_muscles(brbd):
    # Load a predefined model
    model = brbd.Biorbd("../../models/arm26.bioMod")
    muscles = model.muscles
    nmus = len(muscles)

    assert nmus == len(model.internal.muscles())

    # Choose a state (position/velocity) to compute dynamics from
    activations = [0.5] * nmus
    q = [0.1] * model.nb_q
    qdot = [0.1] * model.nb_qdot

    # 1. Using the "low-level" way
    muscles.activations = activations
    muscles.update_geometry(q=q, qdot=qdot)
    tau = muscles.joint_torque()
    np.testing.assert_almost_equal(tau, [-8.99327207, -11.62806138])

    # 2. The "high-level" way, where the kinematics is updated internally (using a different q so we are not accidentally up to date)
    tau = muscles.joint_torque(activations=activations, q=np.array(q) * 2, qdot=qdot)
    np.testing.assert_almost_equal(tau, [-10.11532606, -10.13974674])

    # Validate forces and length jacobian
    forces = muscles.forces()
    jacobian = np.concatenate(muscles.length_jacobian())
    np.testing.assert_almost_equal(-jacobian.T @ forces, [-10.11532606, -10.13974674])

    # Test the muscle forces for internal and explicit kinematics update
    np.testing.assert_almost_equal(
        muscles.forces(), [403.47878369, 341.14054494, 214.28139395, 239.38801482, 201.51684182, 493.98451373]
    )
    np.testing.assert_almost_equal(
        muscles.forces(q=np.array(q) * 2, qdot=qdot),
        [403.47878369, 341.14054494, 214.28139395, 239.38801482, 201.51684182, 493.98451373],
    )
    np.testing.assert_almost_equal(
        muscles.forces(activations=[0.3] * nmus),
        [245.63266637, 225.70738926, 132.04335764, 143.63280889, 117.39623324, 296.81063517],
    )

    # Test activation dot
    initial_activations = [0.5] * nmus
    excitations = np.arange(1, nmus + 1) / nmus  # Varying excitations from 0.1 to 1.0
    activations_dot = muscles.activations_dot(excitations=excitations, activations=initial_activations)
    np.testing.assert_almost_equal(
        activations_dot,
        [-10.416666666666668, -5.208333333333334, 0.0, 13.33333333333333, 26.666666666666668, 40.0],
    )

    # Get an change the properties of a specific muscle
    muscle = muscles[0]
    assert muscle.name == "TRIlong"

    assert muscle.optimal_length == 0.134
    muscle.optimal_length = 0.2
    assert muscle.optimal_length == 0.2

    assert muscle.maximal_isometric_force == 798.52
    muscle.maximal_isometric_force = 1000.0
    assert muscle.maximal_isometric_force == 1000.0

    assert muscle.pcsa == 0.0
    muscle.pcsa = 0.02
    assert muscle.pcsa == 0.02

    assert muscle.tendon_slack_length == 0.143
    muscle.tendon_slack_length = 0.3
    assert muscle.tendon_slack_length == 0.3

    assert muscle.pennation_angle == 0.20943951
    muscle.pennation_angle = 0.1
    assert muscle.pennation_angle == 0.1

    assert muscle.maximal_contraction_velocity == 10.0
    muscle.maximal_contraction_velocity = 15.0
    assert muscle.maximal_contraction_velocity == 15.0

    # The non-updated value is not suppose to reflect the changes properly
    assert muscle.force == muscles.forces()[0]
    np.testing.assert_almost_equal(
        muscles.forces(), [441.00852545, 341.14054494, 214.28139395, 239.38801482, 201.51684182, 493.98451373]
    )
    muscles.update_geometry(q=q, qdot=qdot)
    assert muscle.force == muscles.forces()[0]
    np.testing.assert_almost_equal(
        muscles.forces(), [37.43286925, 349.67505477, 212.67611275, 231.48743683, 194.03986176, 494.72543227]
    )


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_static_optimization(brbd):
    assert brbd.has_static_optimization is True

    # Load a predefined model
    model = brbd.Biorbd("../../models/arm26.bioMod")
    n_frames = 3

    q = []
    qdot = []
    qddot = []
    tau = []
    for i in range(n_frames):
        # The q would typically come from an inverse kinematics analysis
        q.append([0] * model.nb_q)
        # The qdot and qddot would typically come from a numerical differentiation of the kinematics
        qdot.append([0] * model.nb_qdot)
        qddot.append([0] * model.nb_qddot)

        # The tau would typically come from an inverse dynamics analysis as follow
        tau.append(model.inverse_dynamics(q[i], qdot[i], qddot[i]))

    # Proceed with the static optimization. When perform is called, all the frames are processed at once, even though
    # it is a loop. That is so the initial guess is dependent of the previous frame. So the first "frame" of the loop is
    # very long (as it computes everythin). Then, the following frames are very fast (as it only returns the precomputed
    # results)
    optim = brbd.StaticOptimization(model)
    muscle_activations = []
    for value in optim.perform_frames(q, qdot, tau):
        muscle_activations.append(value)
    assert len(muscle_activations) == n_frames
    np.testing.assert_almost_equal(
        muscle_activations[0], [0.00012686, 0.00012018, 0.00089968, 0.00010025, 0.00010027, 0.00010081]
    )
    np.testing.assert_almost_equal(
        muscle_activations[-1], [0.00012686, 0.00012018, 0.00089968, 0.00010025, 0.00010027, 0.00010081]
    )


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_wrapper_model(brbd)
        test_wrapper_segments(brbd)
        test_wrapper_markers(brbd)
        test_wrapper_frames(brbd)
        test_wrapper_dynamics(brbd)
        test_wrapper_external_forces(brbd)
        test_muscles(brbd)
        test_wrapper_kalman_filter(brbd)  # This test is long
        if brbd.has_static_optimization:
            test_static_optimization(brbd)
