import os
import pytest

import numpy as np


brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except ModuleNotFoundError:
    pass


import ezc3d


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_model_creation_from_static(brbd, remove_temporary: bool = True):
    """
    Produces a model from real data
    """
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
        Translations,
        Rotations,
    )

    kinematic_model_file_path = "temporary.bioMod"

    # Create a model holder
    bio_model = BiomechanicalModelReal()

    # The trunk segment
    bio_model.segments["TRUNK"] = SegmentReal(
        translations=Translations.YZ,
        rotations=Rotations.X,
        mesh=MeshReal(((0, 0, 0), (0, 0, 0.53))),
    )
    bio_model.segments["TRUNK"].add_marker(MarkerReal(name="PELVIS", parent_name="TRUNK"))

    # The head segment
    bio_model.segments["HEAD"] = SegmentReal(
        parent_name="TRUNK",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, 0.53)
        ),
        mesh=MeshReal(((0, 0, 0), (0, 0, 0.24))),
    )
    bio_model.segments["HEAD"].add_marker(MarkerReal(name="BOTTOM_HEAD", parent_name="HEAD", position=(0, 0, 0)))
    bio_model.segments["HEAD"].add_marker(MarkerReal(name="TOP_HEAD", parent_name="HEAD", position=(0, 0, 0.24)))
    bio_model.segments["HEAD"].add_marker(MarkerReal(name="HEAD_Z", parent_name="HEAD", position=(0, 0, 0.24)))
    bio_model.segments["HEAD"].add_marker(MarkerReal(name="HEAD_XZ", parent_name="HEAD", position=(0.24, 0, 0.24)))

    # The arm segment
    bio_model.segments["UPPER_ARM"] = SegmentReal(
        parent_name="TRUNK",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, 0.53)
        ),
        rotations=Rotations.X,
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.28))),
    )
    bio_model.segments["UPPER_ARM"].add_marker(MarkerReal(name="SHOULDER", parent_name="UPPER_ARM", position=(0, 0, 0)))
    bio_model.segments["UPPER_ARM"].add_marker(MarkerReal(name="SHOULDER_X", parent_name="UPPER_ARM", position=(1, 0, 0)))
    bio_model.segments["UPPER_ARM"].add_marker(MarkerReal(name="SHOULDER_XY", parent_name="UPPER_ARM", position=(1, 1, 0)))

    bio_model.segments["LOWER_ARM"] = SegmentReal(
        name="LOWER_ARM",
        parent_name="UPPER_ARM",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.28)
        ),
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.27))),
    )
    bio_model.segments["LOWER_ARM"].add_marker(MarkerReal(name="ELBOW", parent_name="LOWER_ARM", position=(0, 0, 0)))
    bio_model.segments["LOWER_ARM"].add_marker(MarkerReal(name="ELBOW_Y", parent_name="LOWER_ARM", position=(0, 1, 0)))
    bio_model.segments["LOWER_ARM"].add_marker(MarkerReal(name="ELBOW_XY", parent_name="LOWER_ARM", position=(1, 1, 0)))

    bio_model.segments["HAND"] = SegmentReal(
        name="HAND",
        parent_name="LOWER_ARM",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.27)
        ),
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.19))),
    )
    bio_model.segments["HAND"].add_marker(MarkerReal(name="WRIST", parent_name="HAND", position=(0, 0, 0)))
    bio_model.segments["HAND"].add_marker(MarkerReal(name="FINGER", parent_name="HAND", position=(0, 0, -0.19)))
    bio_model.segments["HAND"].add_marker(MarkerReal(name="HAND_Y", parent_name="HAND", position=(0, 1, 0)))
    bio_model.segments["HAND"].add_marker(MarkerReal(name="HAND_YZ", parent_name="HAND", position=(0, 1, 1)))

    # The thigh segment
    bio_model.segments["THIGH"] = SegmentReal(
        name="THIGH",
        parent_name="TRUNK",
        rotations=Rotations.X,
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.42))),
    )
    bio_model.segments["THIGH"].add_marker(MarkerReal(name="THIGH_ORIGIN", parent_name="THIGH", position=(0, 0, 0)))
    bio_model.segments["THIGH"].add_marker(MarkerReal(name="THIGH_X", parent_name="THIGH", position=(1, 0, 0)))
    bio_model.segments["THIGH"].add_marker(MarkerReal(name="THIGH_Y", parent_name="THIGH", position=(0, 1, 0)))

    # The shank segment
    bio_model.segments["SHANK"] = SegmentReal(
        name="SHANK",
        parent_name="THIGH",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.42)
        ),
        rotations=Rotations.X,
        mesh=MeshReal(((0, 0, 0), (0, 0, -0.43))),
    )
    bio_model.segments["SHANK"].add_marker(MarkerReal(name="KNEE", parent_name="SHANK", position=(0, 0, 0)))
    bio_model.segments["SHANK"].add_marker(MarkerReal(name="KNEE_Z", parent_name="SHANK", position=(0, 0, 1)))
    bio_model.segments["SHANK"].add_marker(MarkerReal(name="KNEE_XZ", parent_name="SHANK", position=(1, 0, 1)))

    # The foot segment
    bio_model.segments["FOOT"] = SegmentReal(
        name="FOOT",
        parent_name="SHANK",
        segment_coordinate_system=SegmentCoordinateSystemReal.from_euler_and_translation(
            (-np.pi / 2, 0, 0), "xyz", (0, 0, -0.43)
        ),
        rotations=Rotations.X,
        mesh=MeshReal(((0, 0, 0), (0, 0, 0.25))),
    )
    bio_model.segments["FOOT"].add_marker(MarkerReal(name="ANKLE", parent_name="FOOT", position=(0, 0, 0)))
    bio_model.segments["FOOT"].add_marker(MarkerReal(name="TOE", parent_name="FOOT", position=(0, 0, 0.25)))
    bio_model.segments["FOOT"].add_marker(MarkerReal(name="ANKLE_Z", parent_name="FOOT", position=(0, 0, 1)))
    bio_model.segments["FOOT"].add_marker(MarkerReal(name="ANKLE_YZ", parent_name="FOOT", position=(0, 1, 1)))

    # Put the model together, print it and print it to a bioMod file
    bio_model.write(kinematic_model_file_path)

    model = biorbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 25
    np.testing.assert_almost_equal(model.markers(np.zeros((model.nbQ(),)))[-3].to_array(), [0, 0.25, -0.85], decimal=4)

    if remove_temporary:
        os.remove(kinematic_model_file_path)


def write_markers_to_c3d(save_path: str, model):
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


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_model_creation_from_data(brbd, remove_temporary: bool = True):
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
        Translations,
        Rotations,
    )
    from de_leva import DeLevaTable

    kinematic_model_file_path = "temporary.bioMod"
    c3d_file_path = "temporary.c3d"
    test_model_creation_from_static(brbd, remove_temporary=False)

    # Prepare a fake model and a fake static from the previous test
    model = biorbd.Model(kinematic_model_file_path)
    write_markers_to_c3d(c3d_file_path, model)
    os.remove(kinematic_model_file_path)

    # Fill the kinematic chain model
    model = BiomechanicalModel()
    de_leva = DeLevaTable(total_mass=100, sex="female")

    model.segments["TRUNK"] = Segment(
        translations=Translations.YZ,
        rotations=Rotations.X,
        inertia_parameters=de_leva["TRUNK"],
    )
    model.segments["TRUNK"].add_marker(Marker("PELVIS"))

    model.segments["HEAD"] = Segment(
        parent_name="TRUNK",
        segment_coordinate_system=SegmentCoordinateSystem(
            "BOTTOM_HEAD",
            first_axis=Axis(name=Axis.Name.Z, start="BOTTOM_HEAD", end="HEAD_Z"),
            second_axis=Axis(name=Axis.Name.X, start="BOTTOM_HEAD", end="HEAD_XZ"),
            axis_to_keep=Axis.Name.Z,
        ),
        mesh=Mesh(("BOTTOM_HEAD", "TOP_HEAD", "HEAD_Z", "HEAD_XZ", "BOTTOM_HEAD")),
        inertia_parameters=de_leva["HEAD"],
    )
    model.segments["HEAD"].add_marker(Marker("BOTTOM_HEAD"))
    model.segments["HEAD"].add_marker(Marker("TOP_HEAD"))
    model.segments["HEAD"].add_marker(Marker("HEAD_Z"))
    model.segments["HEAD"].add_marker(Marker("HEAD_XZ"))

    model.segments["UPPER_ARM"] = Segment(
        "UPPER_ARM",
        parent_name="TRUNK",
        rotations=Rotations.X,
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="SHOULDER",
            first_axis=Axis(name=Axis.Name.X, start="SHOULDER", end="SHOULDER_X"),
            second_axis=Axis(name=Axis.Name.Y, start="SHOULDER", end="SHOULDER_XY"),
            axis_to_keep=Axis.Name.X,
        ),
        inertia_parameters=de_leva["UPPER_ARM"],
    )
    model.segments["UPPER_ARM"].add_marker(Marker("SHOULDER"))
    model.segments["UPPER_ARM"].add_marker(Marker("SHOULDER_X"))
    model.segments["UPPER_ARM"].add_marker(Marker("SHOULDER_XY"))

    model.segments["LOWER_ARM"] = Segment(
        parent_name="UPPER_ARM",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="ELBOW",
            first_axis=Axis(name=Axis.Name.Y, start="ELBOW", end="ELBOW_Y"),
            second_axis=Axis(name=Axis.Name.X, start="ELBOW", end="ELBOW_XY"),
            axis_to_keep=Axis.Name.Y,
        ),
        inertia_parameters=de_leva["LOWER_ARM"],
    )
    model.segments["LOWER_ARM"].add_marker(Marker("ELBOW"))
    model.segments["LOWER_ARM"].add_marker(Marker("ELBOW_Y"))
    model.segments["LOWER_ARM"].add_marker(Marker("ELBOW_XY"))

    model.segments["HAND"] = Segment(
        parent_name="LOWER_ARM",
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="WRIST",
            first_axis=Axis(name=Axis.Name.Y, start="WRIST", end="HAND_Y"),
            second_axis=Axis(name=Axis.Name.Z, start="WRIST", end="HAND_YZ"),
            axis_to_keep=Axis.Name.Y,
        ),
        inertia_parameters=de_leva["HAND"],
    )
    model.segments["HAND"].add_marker(Marker("WRIST"))
    model.segments["HAND"].add_marker(Marker("FINGER"))
    model.segments["HAND"].add_marker(Marker("HAND_Y"))
    model.segments["HAND"].add_marker(Marker("HAND_YZ"))

    model.segments["THIGH"] = Segment(
        parent_name="TRUNK",
        rotations=Rotations.X,
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="THIGH_ORIGIN",
            first_axis=Axis(name=Axis.Name.X, start="THIGH_ORIGIN", end="THIGH_X"),
            second_axis=Axis(name=Axis.Name.Y, start="THIGH_ORIGIN", end="THIGH_Y"),
            axis_to_keep=Axis.Name.X,
        ),
        inertia_parameters=de_leva["THIGH"],
    )
    model.segments["THIGH"].add_marker(Marker("THIGH_ORIGIN"))
    model.segments["THIGH"].add_marker(Marker("THIGH_X"))
    model.segments["THIGH"].add_marker(Marker("THIGH_Y"))

    model.segments["SHANK"] = Segment(
        parent_name="THIGH",
        rotations=Rotations.X,
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="KNEE",
            first_axis=Axis(name=Axis.Name.Z, start="KNEE", end="KNEE_Z"),
            second_axis=Axis(name=Axis.Name.X, start="KNEE", end="KNEE_XZ"),
            axis_to_keep=Axis.Name.Z,
        ),
        inertia_parameters=de_leva["SHANK"],
    )
    model.segments["SHANK"].add_marker(Marker("KNEE"))
    model.segments["SHANK"].add_marker(Marker("KNEE_Z"))
    model.segments["SHANK"].add_marker(Marker("KNEE_XZ"))

    model.segments["FOOT"] = Segment(
        parent_name="SHANK",
        rotations=Rotations.X,
        segment_coordinate_system=SegmentCoordinateSystem(
            origin="ANKLE",
            first_axis=Axis(name=Axis.Name.Z, start="ANKLE", end="ANKLE_Z"),
            second_axis=Axis(name=Axis.Name.Y, start="ANKLE", end="ANKLE_YZ"),
            axis_to_keep=Axis.Name.Z,
        ),
        inertia_parameters=de_leva["FOOT"],
    )
    model.segments["FOOT"].add_marker(Marker("ANKLE"))
    model.segments["FOOT"].add_marker(Marker("TOE"))
    model.segments["FOOT"].add_marker(Marker("ANKLE_Z"))
    model.segments["FOOT"].add_marker(Marker("ANKLE_YZ"))

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



def test_complex_model():
    from biorbd.examples.python3.modelCreation import complex_model_from_scratch
    mesh_path = os.path.dirname(__file__) + "/../../models/meshFiles/stl/pendulum.STL"
    complex_model_from_scratch(mesh_path=mesh_path)