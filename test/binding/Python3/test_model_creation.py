import os

import numpy as np
import biorbd
from biorbd.model_creation import (
    Axis,
    BiomechanicalModel,
    BiomechanicalModelReal,
    C3dData,
    Marker,
    MarkerReal,
    Mesh,
    MeshReal,
    Segment,
    SegmentReal,
    SegmentCoordinateSystemReal,
    SegmentCoordinateSystem,
)
import ezc3d

from de_leva import DeLevaTable


def test_model_creation_from_static(remove_temporary: bool = True):
    """
    Produces a model from real data
    """

    kinematic_model_file_path = "temporary.bioMod"

    # Create a model holder
    bio_model = BiomechanicalModelReal()

    # The trunk segment
    bio_model["TRUNK"] = SegmentReal(
        translations="yz",
        rotations="x",
        mesh=MeshReal(((0, 0, 0), (0, 0, 0.53))),
    )
    bio_model["TRUNK"].add_marker(MarkerReal(name="PELVIS", parent_name="TRUNK"))

    # The head segment
    bio_model["HEAD"] = SegmentReal(
        parent_name="TRUNK",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, 0.53)
        ),
        mesh=MeshReal(((0, 0, 0), (0, 0, 0.24))),
    )
    bio_model["HEAD"].add_marker(MarkerReal(name="BOTTOM_HEAD", parent_name="HEAD", position=(0, 0, 0)))
    bio_model["HEAD"].add_marker(MarkerReal(name="TOP_HEAD", parent_name="HEAD", position=(0, 0, 0.24)))
    bio_model["HEAD"].add_marker(MarkerReal(name="HEAD_Z", parent_name="HEAD", position=(0, 0, 0.24)))
    bio_model["HEAD"].add_marker(MarkerReal(name="HEAD_XZ", parent_name="HEAD", position=(0.24, 0, 0.24)))

    # The arm segment
    bio_model["UPPER_ARM"] = SegmentReal(
        parent_name="TRUNK",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, 0.53)
        ),
        rotations="x",
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.28))),
    )
    bio_model["UPPER_ARM"].add_marker(MarkerReal(name="SHOULDER", parent_name="UPPER_ARM", position=(0, 0, 0)))
    bio_model["UPPER_ARM"].add_marker(MarkerReal(name="SHOULDER_X", parent_name="UPPER_ARM", position=(1, 0, 0)))
    bio_model["UPPER_ARM"].add_marker(MarkerReal(name="SHOULDER_XY", parent_name="UPPER_ARM", position=(1, 1, 0)))

    bio_model["LOWER_ARM"] = SegmentReal(
        name="LOWER_ARM",
        parent_name="UPPER_ARM",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.28)
        ),
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.27))),
    )
    bio_model["LOWER_ARM"].add_marker(MarkerReal(name="ELBOW", parent_name="LOWER_ARM", position=(0, 0, 0)))
    bio_model["LOWER_ARM"].add_marker(MarkerReal(name="ELBOW_Y", parent_name="LOWER_ARM", position=(0, 1, 0)))
    bio_model["LOWER_ARM"].add_marker(MarkerReal(name="ELBOW_XY", parent_name="LOWER_ARM", position=(1, 1, 0)))

    bio_model["HAND"] = SegmentReal(
        name="HAND",
        parent_name="LOWER_ARM",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.27)
        ),
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.19))),
    )
    bio_model["HAND"].add_marker(MarkerReal(name="WRIST", parent_name="HAND", position=(0, 0, 0)))
    bio_model["HAND"].add_marker(MarkerReal(name="FINGER", parent_name="HAND", position=(0, 0, -0.19)))
    bio_model["HAND"].add_marker(MarkerReal(name="HAND_Y", parent_name="HAND", position=(0, 1, 0)))
    bio_model["HAND"].add_marker(MarkerReal(name="HAND_YZ", parent_name="HAND", position=(0, 1, 1)))

    # The thigh segment
    bio_model["THIGH"] = SegmentReal(
        name="THIGH",
        parent_name="TRUNK",
        rotations="x",
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.42))),
    )
    bio_model["THIGH"].add_marker(MarkerReal(name="THIGH_ORIGIN", parent_name="THIGH", position=(0, 0, 0)))
    bio_model["THIGH"].add_marker(MarkerReal(name="THIGH_X", parent_name="THIGH", position=(1, 0, 0)))
    bio_model["THIGH"].add_marker(MarkerReal(name="THIGH_Y", parent_name="THIGH", position=(0, 1, 0)))

    # The shank segment
    bio_model["SHANK"] = SegmentReal(
        name="SHANK",
        parent_name="THIGH",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.42)
        ),
        rotations="x",
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.43))),
    )
    bio_model["SHANK"].add_marker(MarkerReal(name="KNEE", parent_name="SHANK", position=(0, 0, 0)))
    bio_model["SHANK"].add_marker(MarkerReal(name="KNEE_Z", parent_name="SHANK", position=(0, 0, 1)))
    bio_model["SHANK"].add_marker(MarkerReal(name="KNEE_XZ", parent_name="SHANK", position=(1, 0, 1)))

    # The foot segment
    bio_model["FOOT"] = SegmentReal(
        name="FOOT",
        parent_name="SHANK",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (-np.pi / 2, 0, 0), "xyz", (0, 0, -0.43)
        ),
        rotations="x",
        mesh=MeshReal(((0, 0, 0), (0, 0, 0.25))),
    )
    bio_model["FOOT"].add_marker(MarkerReal(name="ANKLE", parent_name="FOOT", position=(0, 0, 0)))
    bio_model["FOOT"].add_marker(MarkerReal(name="TOE", parent_name="FOOT", position=(0, 0, 0.25)))
    bio_model["FOOT"].add_marker(MarkerReal(name="ANKLE_Z", parent_name="FOOT", position=(0, 0, 1)))
    bio_model["FOOT"].add_marker(MarkerReal(name="ANKLE_YZ", parent_name="FOOT", position=(0, 1, 1)))

    # Put the model together, print it and print it to a bioMod file
    bio_model.write(kinematic_model_file_path)

    model = biorbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 25
    np.testing.assert_almost_equal(model.markers(np.zeros((model.nbQ(),)))[-3].to_array(), [0, 0.25, -0.85], decimal=4)

    if remove_temporary:
        os.remove(kinematic_model_file_path)


def write_markers_to_c3d(save_path: str, model: biorbd.Model):
    q = np.zeros(model.nbQ())
    marker_names = tuple(name.to_string() for name in model.markerNames())
    marker_positions = np.array(tuple(m.to_array() for m in model.markers(q))).T[:, :, np.newaxis]
    c3d = ezc3d.c3d()

    # Fill it with random data
    c3d["parameters"]["POINT"]["RATE"]["value"] = [100]
    c3d["parameters"]["POINT"]["LABELS"]["value"] = marker_names
    c3d["data"]["points"] = marker_positions

    # Write the data
    c3d.write(save_path)


def test_model_creation_from_data(remove_temporary: bool = True):
    kinematic_model_file_path = "temporary.bioMod"
    c3d_file_path = "temporary.c3d"
    test_model_creation_from_static(remove_temporary=False)

    # Prepare a fake model and a fake static from the previous test
    model = biorbd.Model(kinematic_model_file_path)
    write_markers_to_c3d(c3d_file_path, model)
    os.remove(kinematic_model_file_path)

    # Fill the kinematic chain model
    model = BiomechanicalModel()
    de_leva = DeLevaTable(total_mass=100, sex="female")

    model["TRUNK"] = Segment(
        translations="yx",
        rotations="x",
        inertia_parameters=de_leva["TRUNK"],
    )
    model["TRUNK"].add_marker(Marker("PELVIS"))

    model["HEAD"] = Segment(
        parent_name="TRUNK",
        segment_coordinate_system=SegmentCoordinateSystem(
            "BOTTOM_HEAD",
            first_axis=Axis(Axis.Name.Z, start="BOTTOM_HEAD", end="HEAD_Z"),
            second_axis=Axis(Axis.Name.X, start="BOTTOM_HEAD", end="HEAD_XZ"),
            axis_to_keep=Axis.Name.Z,
        ),
        mesh=Mesh(("BOTTOM_HEAD", "TOP_HEAD", "HEAD_Z", "HEAD_XZ", "BOTTOM_HEAD")),
        inertia_parameters=de_leva["HEAD"],
    )
    model["HEAD"].add_marker(Marker("BOTTOM_HEAD"))
    model["HEAD"].add_marker(Marker("TOP_HEAD"))
    model["HEAD"].add_marker(Marker("HEAD_Z"))
    model["HEAD"].add_marker(Marker("HEAD_XZ"))

    model["UPPER_ARM"] = Segment(
        "UPPER_ARM",
        parent_name="TRUNK",
        rotations="x",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="SHOULDER",
            first_axis=Axis(Axis.Name.X, start="SHOULDER", end="SHOULDER_X"),
            second_axis=Axis(Axis.Name.Y, start="SHOULDER", end="SHOULDER_XY"),
            axis_to_keep=Axis.Name.X,
        ),
        inertia_parameters=de_leva["UPPER_ARM"],
    )
    model["UPPER_ARM"].add_marker(Marker("SHOULDER"))
    model["UPPER_ARM"].add_marker(Marker("SHOULDER_X"))
    model["UPPER_ARM"].add_marker(Marker("SHOULDER_XY"))

    model["LOWER_ARM"] = Segment(
        parent_name="UPPER_ARM",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="ELBOW",
            first_axis=Axis(Axis.Name.Y, start="ELBOW", end="ELBOW_Y"),
            second_axis=Axis(Axis.Name.X, start="ELBOW", end="ELBOW_XY"),
            axis_to_keep=Axis.Name.Y,
        ),
        inertia_parameters=de_leva["LOWER_ARM"],
    )
    model["LOWER_ARM"].add_marker(Marker("ELBOW"))
    model["LOWER_ARM"].add_marker(Marker("ELBOW_Y"))
    model["LOWER_ARM"].add_marker(Marker("ELBOW_XY"))

    model["HAND"] = Segment(
        parent_name="LOWER_ARM",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="WRIST",
            first_axis=Axis(Axis.Name.Y, start="WRIST", end="HAND_Y"),
            second_axis=Axis(Axis.Name.Z, start="WRIST", end="HAND_YZ"),
            axis_to_keep=Axis.Name.Y,
        ),
        inertia_parameters=de_leva["HAND"],
    )
    model["HAND"].add_marker(Marker("WRIST"))
    model["HAND"].add_marker(Marker("FINGER"))
    model["HAND"].add_marker(Marker("HAND_Y"))
    model["HAND"].add_marker(Marker("HAND_YZ"))

    model["THIGH"] = Segment(
        parent_name="TRUNK",
        rotations="x",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="THIGH_ORIGIN",
            first_axis=Axis(Axis.Name.X, start="THIGH_ORIGIN", end="THIGH_X"),
            second_axis=Axis(Axis.Name.Y, start="THIGH_ORIGIN", end="THIGH_Y"),
            axis_to_keep=Axis.Name.X,
        ),
        inertia_parameters=de_leva["THIGH"],
    )
    model["THIGH"].add_marker(Marker("THIGH_ORIGIN"))
    model["THIGH"].add_marker(Marker("THIGH_X"))
    model["THIGH"].add_marker(Marker("THIGH_Y"))

    model["SHANK"] = Segment(
        parent_name="THIGH",
        rotations="x",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="KNEE",
            first_axis=Axis(Axis.Name.Z, start="KNEE", end="KNEE_Z"),
            second_axis=Axis(Axis.Name.X, start="KNEE", end="KNEE_XZ"),
            axis_to_keep=Axis.Name.Z,
        ),
        inertia_parameters=de_leva["SHANK"],
    )
    model["SHANK"].add_marker(Marker("KNEE"))
    model["SHANK"].add_marker(Marker("KNEE_Z"))
    model["SHANK"].add_marker(Marker("KNEE_XZ"))

    model["FOOT"] = Segment(
        parent_name="SHANK",
        rotations="x",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="ANKLE",
            first_axis=Axis(Axis.Name.Z, start="ANKLE", end="ANKLE_Z"),
            second_axis=Axis(Axis.Name.Y, start="ANKLE", end="ANKLE_YZ"),
            axis_to_keep=Axis.Name.Z,
        ),
        inertia_parameters=de_leva["FOOT"],
    )
    model["FOOT"].add_marker(Marker("ANKLE"))
    model["FOOT"].add_marker(Marker("TOE"))
    model["FOOT"].add_marker(Marker("ANKLE_Z"))
    model["FOOT"].add_marker(Marker("ANKLE_YZ"))

    # Put the model together, print it and print it to a bioMod file
    model.write(kinematic_model_file_path, C3dData(c3d_file_path))

    model = biorbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 25
    np.testing.assert_almost_equal(model.markers(np.zeros((model.nbQ(),)))[-3].to_array(), [0, 0.25, -0.85], decimal=4)

    if remove_temporary:
        os.remove(kinematic_model_file_path)
        os.remove(c3d_file_path)
