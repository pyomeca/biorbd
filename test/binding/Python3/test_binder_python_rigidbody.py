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
    ranges = m.segment(0).ranges()
    assert(ranges[0].min() == -10)
    assert(ranges[0].max() == 10)
    assert(ranges[1].min() == -10)
    assert(ranges[1].max() == 10)
    assert(ranges[2].min() == -pi)
    assert(ranges[2].max() == pi)

    # BrasD
    ranges = m.segment(3).ranges()
    assert(ranges[0].min() == -pi)
    assert(ranges[0].max() == pi)
    assert(ranges[1].min() == 0)
    assert(ranges[1].max() == pi)

    # BrasG
    ranges = m.segment(4).ranges()
    assert(ranges[0].min() == -pi)
    assert(ranges[0].max() == pi)
    assert(ranges[1].min() == 0)
    assert(ranges[1].max() == pi)

    # CuisseD
    ranges = m.segment(5).ranges()
    assert(ranges[0].min() == -pi / 12)
    assert(ranges[0].max() == pi / 2 + pi / 3)

    # JambeD
    ranges = m.segment(6).ranges()
    assert(ranges[0].min() == -pi / 2 - pi / 6)
    assert(ranges[0].max() == 0)

    # PiedD
    ranges = m.segment(7).ranges()
    assert(ranges[0].min() == -pi / 2)
    assert(ranges[0].max() == pi / 2)

    # CuisseG
    ranges = m.segment(8).ranges()
    assert(ranges[0].min() == -pi / 12)
    assert(ranges[0].max() == pi / 2 + pi / 3)

    # JambeG
    ranges = m.segment(9).ranges()
    assert(ranges[0].min() == -pi / 2 - pi / 6)
    assert(ranges[0].max() == 0)

    # PiedG
    ranges = m.segment(10).ranges()
    assert(ranges[0].min() == -pi / 2)
    assert(ranges[0].max() == pi / 2)
