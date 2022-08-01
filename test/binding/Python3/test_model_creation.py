import os

import numpy as np
import biorbd
from biorbd import Marker, Segment, KinematicChain, RT, Axis
import ezc3d


def test_model_creation_from_static(remove_temporary: bool = True):
    kinematic_model_file_path = "temporary.bioMod"

    # The trunk segment
    trunk_marker_pelvis = Marker(name="PELVIS", parent_name="TRUNK")
    trunk = Segment(
        name="TRUNK",
        translations="yz",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, 0.53)),
        markers=(trunk_marker_pelvis,),
    )

    # The head segment
    bottom_head_marker_head = Marker(name="BOTTOM_HEAD", parent_name="HEAD", position=(0, 0, 0))
    top_head_marker_head = Marker(name="TOP_HEAD", parent_name="HEAD", position=(0, 0, 0.24))
    head = Segment(
        name="HEAD",
        parent_name="TRUNK",
        rt="0 0 0 xyz 0 0 0.53",
        mesh=((0, 0, 0), (0, 0, 0.24)),
        markers=(bottom_head_marker_head, top_head_marker_head,),
    )

    # The arm segment
    shoulder_marker = Marker(name="SHOULDER", parent_name="UPPER_ARM", position=(0, 0, 0))
    upper_arm = Segment(
        name="UPPER_ARM",
        parent_name=trunk.name,
        rt="0 0 0 xyz 0 0 0.53",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.28)),
        markers=(shoulder_marker,),
    )

    elbow_marker = Marker(name="ELBOW", parent_name="LOWER_ARM", position=(0, 0, 0))
    lower_arm = Segment(
        name="LOWER_ARM",
        parent_name=upper_arm.name,
        rt="0 0 0 xyz 0 0 -0.28",
        mesh=((0, 0, 0), (0, 0, -0.27)),
        markers=(elbow_marker,),
    )

    wrist_marker = Marker(name="WRIST", parent_name="HAND", position=(0, 0, 0))
    finger_marker = Marker(name="FINGER", parent_name="HAND", position=(0, 0, -0.19))
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
    knee_marker = Marker(name="KNEE", parent_name="SHANK", position=(0, 0, 0))
    shank = Segment(
        name="SHANK",
        parent_name=thigh.name,
        rt="0 0 0 xyz 0 0 -0.42",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.43)),
        markers=(knee_marker,),
    )

    # The foot segment
    ankle_marker = Marker(name="ANKLE", parent_name="FOOT", position=(0, 0, 0))
    toe_marker = Marker(name="TOE", parent_name="FOOT", position=(0, 0, 0.25))
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
    assert model.nbMarkers() == 10
    assert (model.markers(np.zeros((model.nbQ(), )))[-1].to_array() == [0, 0, -0.85]).all()

    if remove_temporary:
        os.remove(kinematic_model_file_path)


def write_markers_to_c3d(save_path: str, model: biorbd.Model):
    q = np.zeros(model.nbQ())
    marker_names = tuple(name.to_string() for name in model.markerNames())
    marker_positions = np.array(tuple(m.to_array() for m in model.markers(q))).T[:, :, np.newaxis]
    c3d = ezc3d.c3d()

    # Fill it with random data
    c3d['parameters']['POINT']['RATE']['value'] = [100]
    c3d['parameters']['POINT']['LABELS']['value'] = marker_names
    c3d['data']['points'] = marker_positions

    # Write the data
    c3d.write(save_path)


def test_model_creation_from_data(remove_temporary: bool = True):
    kinematic_model_file_path = "temporary.bioMod"
    c3d_file_path = "temporary.c3d"
    test_model_creation_from_static(remove_temporary=False)

    model = biorbd.Model(kinematic_model_file_path)
    write_markers_to_c3d(c3d_file_path, model)
    os.remove(kinematic_model_file_path)
    data = ezc3d.c3d(c3d_file_path)

    # The trunk segment
    trunk_marker_pelvis = Marker.from_data(data, name="PELVIS", parent_name="TRUNK")
    trunk = Segment(
        name="TRUNK",
        translations="yz",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, 0.53)),
        markers=(trunk_marker_pelvis,),
    )

    # The head segment
    bottom_head_marker_head = Marker.from_data(data, name="BOTTOM_HEAD", parent_name="HEAD")
    top_head_marker_head = Marker.from_data(data, name="TOP_HEAD", parent_name="HEAD")
    origin_head = bottom_head_marker_head
    x_axis_head = origin_head + (0.1, 0, 0)  # Purposefully not 1 so it tests the norm
    y_axis_head = origin_head + (0.1, 0.1, 0)   # Purposefully not perpendicular so it tests realigning
    head = Segment(
        name="HEAD",
        parent_name="TRUNK",
        rt=RT.from_data(
            data,
            origin=origin_head,
            axes=(
                Axis(Axis.Name.X, start=origin_head, end=x_axis_head),
                Axis(Axis.Name.Y, start=origin_head, end=y_axis_head),
                Axis.Name.X,
            ),
        ),
        mesh=((0, 0, 0), (0, 0, 0.24)),
        markers=(bottom_head_marker_head, top_head_marker_head,),
    )

    # The arm segment
    shoulder_marker = Marker.from_data(data, name="SHOULDER", parent_name="UPPER_ARM")
    origin_shoulder = shoulder_marker
    x_axis_shoulder = origin_shoulder + (0.1, 0, 0)  # Purposefully not 1 so it tests the norm
    z_axis_shoulder = origin_shoulder + (0.1, 0, 0.1)   # Purposefully not perpendicular so it tests realigning
    upper_arm = Segment(
        name="UPPER_ARM",
        parent_name=trunk.name,
        rt=RT.from_data(
            data,
            origin=origin_shoulder,
            axes=(
                Axis(Axis.Name.X, start=origin_shoulder, end=x_axis_shoulder),
                Axis(Axis.Name.Z, start=origin_shoulder, end=z_axis_shoulder),
                Axis.Name.X,
            ),
        ),
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.28)),
        markers=(shoulder_marker,),
    )

    elbow_marker = Marker.from_data(data, name="ELBOW", parent_name="LOWER_ARM")
    origin_elbow = elbow_marker - origin_shoulder
    y_axis_elbow = origin_elbow + (0, 0.1, 0)  # Purposefully not 1 so it tests the norm
    x_axis_elbow = origin_elbow + (0.1, 0.1, 0)   # Purposefully not perpendicular so it tests realigning
    lower_arm = Segment(
        name="LOWER_ARM",
        parent_name=upper_arm.name,
        rt=RT.from_data(
            data,
            origin=origin_shoulder,
            axes=(
                Axis(Axis.Name.Y, start=origin_elbow, end=y_axis_elbow),
                Axis(Axis.Name.X, start=origin_elbow, end=x_axis_elbow),
                Axis.Name.Y,
            ),
        ),
        mesh=((0, 0, 0), (0, 0, -0.27)),
        markers=(elbow_marker,),
    )

    wrist_marker = Marker.from_data(data, name="WRIST", parent_name="HAND")
    finger_marker = Marker.from_data(data, name="FINGER", parent_name="HAND")
    origin_wrist = wrist_marker - origin_elbow
    y_axis_wrist = origin_wrist + (0, 0.1, 0)  # Purposefully not 1 so it tests the norm
    z_axis_wrist = origin_wrist + (0, 0.1, 0.1)   # Purposefully not perpendicular so it tests realigning
    hand = Segment(
        name="HAND",
        parent_name=lower_arm.name,
        rt=RT.from_data(
            data,
            origin=origin_shoulder,
            axes=(
                Axis(Axis.Name.Y, start=origin_wrist, end=y_axis_wrist),
                Axis(Axis.Name.Z, start=origin_wrist, end=z_axis_wrist),
                Axis.Name.Y,
            ),
        ),
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
    knee_marker = Marker.from_data(data, name="KNEE", parent_name="SHANK")
    origin_knee = knee_marker
    z_axis_knee = origin_knee + (0, 0, 0.1)  # Purposefully not 1 so it tests the norm
    x_axis_knee = origin_knee + (0.1, 0, 0.1)   # Purposefully not perpendicular so it tests realigning
    shank = Segment(
        name="SHANK",
        parent_name=thigh.name,
        rt=RT.from_data(
            data,
            origin=origin_knee,
            axes=(
                Axis(Axis.Name.Z, start=origin_knee, end=z_axis_knee),
                Axis(Axis.Name.X, start=origin_knee, end=x_axis_knee),
                Axis.Name.Z,
            ),
        ),
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.43)),
        markers=(knee_marker,),
    )

    # The foot segment
    ankle_marker = Marker.from_data(data, name="ANKLE", parent_name="FOOT")
    toe_marker = Marker.from_data(data, name="TOE", parent_name="FOOT")
    origin_ankle = ankle_marker - origin_knee
    z_axis_ankle = origin_ankle + (0, 0, 0.1)  # Purposefully not 1 so it tests the norm
    y_axis_ankle = origin_ankle + (0, 0.1, 0.1)   # Purposefully not perpendicular so it tests realigning
    foot = Segment(
        name="FOOT",
        parent_name=shank.name,
        rt=RT.from_data(
            data,
            origin=origin_ankle,
            axes=(
                Axis(Axis.Name.Z, start=origin_ankle, end=z_axis_ankle),
                Axis(Axis.Name.Y, start=origin_ankle, end=y_axis_ankle),
                Axis.Name.Z,
            ),
        ),
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
    assert model.nbMarkers() == 10
    assert (model.markers(np.zeros((model.nbQ(), )))[-1].to_array() == [0, 0, -0.85]).all()

    if remove_temporary:
        os.remove(kinematic_model_file_path)
        os.remove(c3d_file_path)

