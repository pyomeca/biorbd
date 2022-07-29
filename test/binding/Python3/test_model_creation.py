import os

import numpy as np
import biorbd
from biorbd import Marker, Segment, KinematicChain


def test_model_creation():
    kinematic_model_file_path = "temporary.bioMod"

    # The trunk segment
    trunk_marker_pelvis = Marker(
        name="PELVIS",
        parent_name="TRUNK",
        position=(0, 0, 0),
    )
    trunk = Segment(
        name="TRUNK",
        translations="yz",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, 0.53)),
        markers=(trunk_marker_pelvis,),
    )

    # The head segment
    top_head_marker_head = Marker(
        name="TOP_HEAD",
        parent_name="HEAD",
        position=(0, 0, 0.24),
    )
    head = Segment(
        name="HEAD",
        parent_name="TRUNK",
        rt="0 0 0 xyz 0 0 0.53",
        mesh=((0, 0, 0), (0, 0, 0.24)),
        markers=(top_head_marker_head,),
    )

    # The arm segment
    shoulder_marker = Marker(
        name="SHOULDER",
        parent_name="UPPER_ARM",
        position=(0, 0, 0),
    )
    upper_arm = Segment(
        name="UPPER_ARM",
        parent_name=trunk.name,
        rt="0 0 0 xyz 0 0 0.53",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.28)),
        markers=(shoulder_marker,),
    )

    elbow_marker = Marker(
        name="ELBOW",
        parent_name="LOWER_ARM",
        position=(0, 0, 0),
    )
    lower_arm = Segment(
        name="LOWER_ARM",
        parent_name=upper_arm.name,
        rt="0 0 0 xyz 0 0 -0.28",
        mesh=((0, 0, 0), (0, 0, -0.27)),
        markers=(elbow_marker,),
    )

    wrist_marker = Marker(
        name="WRIST",
        parent_name="HAND",
        position=(0, 0, 0),
    )
    finger_marker = Marker(
        name="FINGER",
        parent_name="HAND",
        position=(0, 0, -0.19),
    )
    hand = Segment(
        name="HAND",
        parent_name=lower_arm.name,
        rt="0 0 0 xyz 0 0 -0.27",
        mesh=((0, 0, 0), (0, 0, -0.19)),
        markers=(wrist_marker, finger_marker)
    )

    # The thigh segment
    thigh = Segment(
        name="THIGH",
        parent_name=trunk.name,
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.42)),
    )

    # The shank segment
    knee_marker = Marker(
        name="KNEE",
        parent_name="SHANK",
        position=(0, 0, 0),
    )
    shank = Segment(
        name="SHANK",
        parent_name=thigh.name,
        rt="0 0 0 xyz 0 0 -0.42",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.43)),
        markers=(knee_marker,),
    )

    # The foot segment
    ankle_marker = Marker(
        name="ANKLE",
        parent_name="FOOT",
        position=(0, 0, 0),
    )
    toe_marker = Marker(
        name="TOE",
        parent_name="FOOT",
        position=(0, 0, 0.25),
    )
    foot = Segment(
        name="FOOT",
        parent_name=shank.name,
        rt="0 0 0 xyz 0 0 -0.43",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, 0.25)),
        markers=(ankle_marker, toe_marker,),
    )

    # Put the model together, print it and print it to a bioMod file
    kinematic_chain = KinematicChain(segments=(trunk, head, upper_arm, lower_arm, hand, thigh, shank, foot))
    kinematic_chain.write(kinematic_model_file_path)

    model = biorbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 9
    assert (model.markers(np.zeros((model.nbQ(), )))[-1].to_array() == [0, 0, -0.6]).all()

    os.remove(kinematic_model_file_path)
