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
