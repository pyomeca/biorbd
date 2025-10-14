import re

import numpy as np
import pytest

from .utils import evaluate, brbd_to_test


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_external_forces(brbd):
    # Load a predefined model
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    # Segment on which the external force will be applied
    segment = model.segments[0]

    # Testing failing cases
    with pytest.raises(ValueError, match=re.escape("The input vector must be of size 3 (force) or 6 (spatial vector)")):
        model.external_force_set.add(segment_name=segment.name, force=[0, 0], point_of_application=[0, 0, 0])
    with pytest.raises(ValueError, match=re.escape("The input vector must be of size 3 (force) or 6 (spatial vector)")):
        model.external_force_set.add(segment_name=segment.name, force=[0, 0, 0, 0], point_of_application=[0, 0, 0])
    with pytest.raises(
        ValueError,
        match="The point of application must be provided when adding a force in the world frame",
    ):
        model.external_force_set.add(segment_name=segment.name, force=[0, 0, 9.81 * float(segment.mass)])
    with pytest.raises(
        ValueError,
        match=re.escape(
            "Adding a force in local frame is not implemented (and probably not what you want). "
            "You probably want to add a spatial vector in the local frame instead or a force in the world frame."
        ),
    ):
        model.external_force_set.add(
            segment_name=segment.name,
            force=[0, 0, 9.81 * float(segment.mass)],
            frame_of_reference=brbd.ExternalForceSet.Frame.LOCAL,
            point_of_application=[0, 0, 0],
        )
    with pytest.raises(
        ValueError,
        match="The point of application must be provided when adding a spatial vector in the local frame",
    ):
        model.external_force_set.add(
            segment_name=segment.name,
            force=[0, 0, 0, 0, 0, 9.81 * float(segment.mass)],
            frame_of_reference=brbd.ExternalForceSet.Frame.LOCAL,
        )

    # Computing forward dynamics as reference
    q = [0] * model.nb_q
    qdot = [0] * model.nb_qdot
    tau = [0] * model.nb_tau
    ref_qddot = evaluate(
        brbd, model.forward_dynamics, q=q, qdot=qdot, tau=tau, ignore_contacts=True, ignore_external_forces=True
    )
    np.testing.assert_array_almost_equal(
        ref_qddot,
        [0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    )

    # Test adding one external force to the model
    model.external_force_set.add(
        segment_name=segment.name, force=[0, 0, 9.81 * float(segment.mass)], point_of_application=[0, 0, 0]
    )
    np.testing.assert_array_almost_equal(
        evaluate(brbd, model.forward_dynamics, q=q, qdot=qdot, tau=tau, ignore_contacts=False),
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
        evaluate(brbd, model.forward_dynamics, q=q, qdot=qdot, tau=tau, ignore_contacts=True),
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
        evaluate(brbd, model.forward_dynamics, q=q, qdot=qdot, tau=tau, ignore_contacts=False),
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
        evaluate(brbd, model.forward_dynamics, q=q, qdot=qdot, tau=tau, ignore_contacts=True),
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
    np.testing.assert_array_almost_equal(
        evaluate(brbd, model.forward_dynamics, q=q, qdot=qdot, tau=tau, ignore_contacts=True), ref_qddot
    )
