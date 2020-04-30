"""
Test for file IO
"""
from pathlib import Path

import numpy as np
import pytest

import biorbd


def test_load_model():
    biorbd.Model('../../models/pyomecaman.bioMod')


def test_dof_ranges():
    m = biorbd.Model('../../models/pyomecaman.bioMod')
    pi = 3.14159265358979323846

    # Pelvis
    QRanges = m.segment(0).QRanges()
    assert(QRanges[0].min() == -10)
    assert(QRanges[0].max() == 10)
    assert(QRanges[1].min() == -10)
    assert(QRanges[1].max() == 10)
    assert(QRanges[2].min() == -pi)
    assert(QRanges[2].max() == pi)

    # BrasD
    QRanges = m.segment(3).QRanges()
    assert(QRanges[0].min() == -pi)
    assert(QRanges[0].max() == pi)
    assert(QRanges[1].min() == 0)
    assert(QRanges[1].max() == pi)

    # BrasG
    QRanges = m.segment(4).QRanges()
    assert(QRanges[0].min() == -pi)
    assert(QRanges[0].max() == pi)
    assert(QRanges[1].min() == 0)
    assert(QRanges[1].max() == pi)

    # CuisseD
    QRanges = m.segment(5).QRanges()
    assert(QRanges[0].min() == -pi / 12)
    assert(QRanges[0].max() == pi / 2 + pi / 3)

    # JambeD
    QRanges = m.segment(6).QRanges()
    assert(QRanges[0].min() == -pi / 2 - pi / 6)
    assert(QRanges[0].max() == 0)

    # PiedD
    QRanges = m.segment(7).QRanges()
    assert(QRanges[0].min() == -pi / 2)
    assert(QRanges[0].max() == pi / 2)

    # CuisseG
    QRanges = m.segment(8).QRanges()
    assert(QRanges[0].min() == -pi / 12)
    assert(QRanges[0].max() == pi / 2 + pi / 3)

    # JambeG
    QRanges = m.segment(9).QRanges()
    assert(QRanges[0].min() == -pi / 2 - pi / 6)
    assert(QRanges[0].max() == 0)

    # PiedG
    QRanges = m.segment(10).QRanges()
    assert(QRanges[0].min() == -pi / 2)
    assert(QRanges[0].max() == pi / 2)
    
def test_forward_dynamics():
    m = biorbd.Model("../../models/pyomecaman_withActuators.bioMod")

    q = np.array([i*1.1 for i in range(m.nbQ())])
    qdot = np.array([i*1.1 for i in range(m.nbQ())])
    tau = np.array([i*1.1 for i in range(m.nbQ())])
    
    qddot = m.ForwardDynamics(q, qdot, tau).to_array()
    qddot_expected = np.array(
        [
            20.554883896960259, -22.317642013324736, -77.406439058256126, 
            17.382961188212313, -63.426361095191858, 93.816468824985876, 
            106.46105024484631, 95.116641811710167, -268.1961283528546, 
            2680.3632159799949, -183.4582596257801, 755.89411812405604,
            163.60239754283589
        ]
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)
    
    # With external forces
    f_ext = biorbd.VecBiorbdSpatialVector(
        [
            biorbd.SpatialVector(11.1, 22.2, 33.3, 44.4, 55.5, 66.6), 
            biorbd.SpatialVector(11.1*2, 22.2*2, 33.3*2, 44.4*2, 55.5*2, 66.6*2)
        ]
    )

    qddot = m.ForwardDynamics(q, qdot, tau, f_ext).to_array()
    qddot_expected = np.array(
        [
            8.8871711208009998, -13.647827029817943, -33.606145294752132, 
            16.922669487341341, -21.882821189868423, 41.15364990805439, 
            68.892537246574463, -324.59756885799197, -447.99217990207387, 
            18884.241415786601, -331.24622725851572, 1364.7620674666462,
            3948.4748602722384
        ]
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)
