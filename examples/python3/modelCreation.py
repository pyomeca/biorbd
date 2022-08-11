import os

import numpy as np
import biorbd
from biorbd import Marker, Segment, KinematicChain, RT, Axis, KinematicModelGeneric
import ezc3d

#
# This examples shows how to
#     1. Create a model from scratch using specified dimensions (model_creation_from_static)
#     1. Create a model from scratch using a template with marker names (model_creation_from_data)
#
# Please note that this example will work only with the Eigen backend
#


def model_creation_from_static(remove_temporary: bool = True):
    """
    We define a new model by feeding in the actual dimension and position of the model
    Please note that a bunch of useless markers are defined, this is for the other model creation which needs them
    to define the RT matrices
    """
    kinematic_model_file_path = "temporary.bioMod"

    # The trunk segment
    trunk = Segment(
        name="TRUNK",
        translations="yz",
        rotations="x",
        mesh=((0, 0, 0), (0, 0, 0.53)),
    )
    trunk.add_marker(Marker(name="PELVIS", parent_name="TRUNK"))

    # The head segment
    head = Segment(
        name="HEAD",
        parent_name="TRUNK",
        rt=RT.from_euler_and_translation((0, 0, 0), "xyz", (0, 0, 0.53)),
        mesh=((0, 0, 0), (0, 0, 0.24)),
    )
    head.add_marker(Marker(name="BOTTOM_HEAD", parent_name="HEAD", position=(0, 0, 0)))
    head.add_marker(Marker(name="TOP_HEAD", parent_name="HEAD", position=(0, 0, 0.24)))
    head.add_marker(Marker(name="HEAD_Z", parent_name="HEAD", position=(0, 0, 0.24)))
    head.add_marker(Marker(name="HEAD_XZ", parent_name="HEAD", position=(0.24, 0, 0.24)))

    # The arm segment
    upper_arm = Segment(
        name="UPPER_ARM",
        parent_name=trunk.name,
        rt=RT.from_euler_and_translation((0, 0, 0), "xyz", (0, 0, 0.53)),
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.28)),
    )
    upper_arm.add_marker(Marker(name="SHOULDER", parent_name="UPPER_ARM", position=(0, 0, 0)))
    upper_arm.add_marker(Marker(name="SHOULDER_X", parent_name="UPPER_ARM", position=(1, 0, 0)))
    upper_arm.add_marker(Marker(name="SHOULDER_XY", parent_name="UPPER_ARM", position=(1, 1, 0)))

    lower_arm = Segment(
        name="LOWER_ARM",
        parent_name=upper_arm.name,
        rt=RT.from_euler_and_translation((0, 0, 0), "xyz", (0, 0, -0.28)),
        mesh=((0, 0, 0), (0, 0, -0.27)),
    )
    lower_arm.add_marker(Marker(name="ELBOW", parent_name="LOWER_ARM", position=(0, 0, 0)))
    lower_arm.add_marker(Marker(name="ELBOW_Y", parent_name="LOWER_ARM", position=(0, 1, 0)))
    lower_arm.add_marker(Marker(name="ELBOW_XY", parent_name="LOWER_ARM", position=(1, 1, 0)))

    hand = Segment(
        name="HAND",
        parent_name=lower_arm.name,
        rt=RT.from_euler_and_translation((0, 0, 0), "xyz", (0, 0, -0.27)),
        mesh=((0, 0, 0), (0, 0, -0.19)),
    )
    hand.add_marker(Marker(name="WRIST", parent_name="HAND", position=(0, 0, 0)))
    hand.add_marker(Marker(name="FINGER", parent_name="HAND", position=(0, 0, -0.19)))
    hand.add_marker(Marker(name="HAND_Y", parent_name="HAND", position=(0, 1, 0)))
    hand.add_marker(Marker(name="HAND_YZ", parent_name="HAND", position=(0, 1, 1)))

    # The thigh segment
    thigh = Segment(
        name="THIGH",
        parent_name=trunk.name,
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.42)),
    )
    thigh.add_marker(Marker(name="THIGH_ORIGIN", parent_name="THIGH", position=(0, 0, 0)))
    thigh.add_marker(Marker(name="THIGH_X", parent_name="THIGH", position=(1, 0, 0)))
    thigh.add_marker(Marker(name="THIGH_Y", parent_name="THIGH", position=(0, 1, 0)))

    # The shank segment
    shank = Segment(
        name="SHANK",
        parent_name=thigh.name,
        rt=RT.from_euler_and_translation((0, 0, 0), "xyz", (0, 0, -0.42)),
        rotations="x",
        mesh=((0, 0, 0), (0, 0, -0.43)),
    )
    shank.add_marker(Marker(name="KNEE", parent_name="SHANK", position=(0, 0, 0)))
    shank.add_marker(Marker(name="KNEE_Z", parent_name="SHANK", position=(0, 0, 1)))
    shank.add_marker(Marker(name="KNEE_XZ", parent_name="SHANK", position=(1, 0, 1)))

    # The foot segment
    foot = Segment(
        name="FOOT",
        parent_name=shank.name,
        rt=RT.from_euler_and_translation((-np.pi / 2, 0, 0), "xyz", (0, 0, -0.43)),
        rotations="x",
        mesh=((0, 0, 0), (0, 0, 0.25)),
    )
    foot.add_marker(Marker(name="ANKLE", parent_name="FOOT", position=(0, 0, 0)))
    foot.add_marker(Marker(name="TOE", parent_name="FOOT", position=(0, 0, 0.25)))
    foot.add_marker(Marker(name="ANKLE_Z", parent_name="FOOT", position=(0, 0, 1)))
    foot.add_marker(Marker(name="ANKLE_YZ", parent_name="FOOT", position=(0, 1, 1)))

    # Put the model together, print it and print it to a bioMod file
    kinematic_chain = KinematicChain(segments=(trunk, head, upper_arm, lower_arm, hand, thigh, shank, foot))
    kinematic_chain.write(kinematic_model_file_path)

    # Do some test to verify that the model was properly created
    model = biorbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 25
    np.testing.assert_almost_equal(model.markers(np.zeros((model.nbQ(),)))[-3].to_array(), [0, 0.25, -0.85], decimal=4)

    # Clean up our mess
    if remove_temporary:
        os.remove(kinematic_model_file_path)


def model_creation_from_data(remove_temporary: bool = True):
    """
    We are using the previous model to define a new model based on the position of the markers
    """

    def write_markers_to_c3d(save_path: str, model: biorbd.Model):
        """
        Write data to a c3d file
        """
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

    kinematic_model_file_path = "temporary.bioMod"
    c3d_file_path = "temporary.c3d"
    model_creation_from_static(remove_temporary=False)

    # Prepare a fake model and a fake static from the previous test
    model = biorbd.Model(kinematic_model_file_path)
    write_markers_to_c3d(c3d_file_path, model)
    os.remove(kinematic_model_file_path)

    # Fill the kinematic chain model
    model = KinematicModelGeneric()

    model.add_segment(name="TRUNK", translations="yx", rotations="x")
    model.add_marker("TRUNK", "PELVIS")

    model.add_segment(name="HEAD")
    model.set_rt(
        "HEAD",
        origin_markers="BOTTOM_HEAD",
        first_axis_name=Axis.Name.Z,
        first_axis_markers=("BOTTOM_HEAD", "HEAD_Z"),
        second_axis_name=Axis.Name.X,
        second_axis_markers=("BOTTOM_HEAD", "HEAD_XZ"),
        axis_to_keep=Axis.Name.Z,
    )
    model.add_marker("HEAD", "BOTTOM_HEAD")
    model.add_marker("HEAD", "TOP_HEAD")
    model.add_marker("HEAD", "HEAD_Z")
    model.add_marker("HEAD", "HEAD_XZ")

    model.add_segment("UPPER_ARM", parent_name="TRUNK", rotations="x")
    model.set_rt(
        "UPPER_ARM",
        origin_markers="SHOULDER",
        first_axis_name=Axis.Name.X,
        first_axis_markers=("SHOULDER", "SHOULDER_X"),
        second_axis_name=Axis.Name.Y,
        second_axis_markers=("SHOULDER", "SHOULDER_XY"),
        axis_to_keep=Axis.Name.X,
    )
    model.add_marker("UPPER_ARM", "SHOULDER")
    model.add_marker("UPPER_ARM", "SHOULDER_X")
    model.add_marker("UPPER_ARM", "SHOULDER_XY")

    model.add_segment("LOWER_ARM", parent_name="UPPER_ARM")
    model.set_rt(
        "LOWER_ARM",
        origin_markers="ELBOW",
        first_axis_name=Axis.Name.Y,
        first_axis_markers=("ELBOW", "ELBOW_Y"),
        second_axis_name=Axis.Name.X,
        second_axis_markers=("ELBOW", "ELBOW_XY"),
        axis_to_keep=Axis.Name.Y,
    )
    model.add_marker("LOWER_ARM", "ELBOW")
    model.add_marker("LOWER_ARM", "ELBOW_Y")
    model.add_marker("LOWER_ARM", "ELBOW_XY")

    model.add_segment("HAND", parent_name="LOWER_ARM")
    model.set_rt(
        "HAND",
        origin_markers="WRIST",
        first_axis_name=Axis.Name.Y,
        first_axis_markers=("WRIST", "HAND_Y"),
        second_axis_name=Axis.Name.Z,
        second_axis_markers=("WRIST", "HAND_YZ"),
        axis_to_keep=Axis.Name.Y,
    )
    model.add_marker("HAND", "WRIST")
    model.add_marker("HAND", "FINGER")
    model.add_marker("HAND", "HAND_Y")
    model.add_marker("HAND", "HAND_YZ")

    model.add_segment("THIGH", parent_name="TRUNK", rotations="x")
    model.set_rt(
        "THIGH",
        origin_markers="THIGH_ORIGIN",
        first_axis_name=Axis.Name.X,
        first_axis_markers=("THIGH_ORIGIN", "THIGH_X"),
        second_axis_name=Axis.Name.Y,
        second_axis_markers=("THIGH_ORIGIN", "THIGH_Y"),
        axis_to_keep=Axis.Name.X,
    )
    model.add_marker("THIGH", "THIGH_ORIGIN")
    model.add_marker("THIGH", "THIGH_X")
    model.add_marker("THIGH", "THIGH_Y")

    model.add_segment("SHANK", parent_name="THIGH", rotations="x")
    model.set_rt(
        "SHANK",
        origin_markers="KNEE",
        first_axis_name=Axis.Name.Z,
        first_axis_markers=("KNEE", "KNEE_Z"),
        second_axis_name=Axis.Name.X,
        second_axis_markers=("KNEE", "KNEE_XZ"),
        axis_to_keep=Axis.Name.Z,
    )
    model.add_marker("SHANK", "KNEE")
    model.add_marker("SHANK", "KNEE_Z")
    model.add_marker("SHANK", "KNEE_XZ")

    model.add_segment("FOOT", parent_name="SHANK", rotations="x")
    model.set_rt(
        "FOOT",
        origin_markers="ANKLE",
        first_axis_name=Axis.Name.Z,
        first_axis_markers=("ANKLE", "ANKLE_Z"),
        second_axis_name=Axis.Name.Y,
        second_axis_markers=("ANKLE", "ANKLE_YZ"),
        axis_to_keep=Axis.Name.Z,
    )
    model.add_marker("FOOT", "ANKLE")
    model.add_marker("FOOT", "TOE")
    model.add_marker("FOOT", "ANKLE_Z")
    model.add_marker("FOOT", "ANKLE_YZ")

    # Put the model together, print it and print it to a bioMod file
    model.generate_personalized(c3d_file_path, kinematic_model_file_path)

    # Do some test to verify that the model was properly created
    model = biorbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 25
    np.testing.assert_almost_equal(model.markers(np.zeros((model.nbQ(),)))[-3].to_array(), [0, 0.25, -0.85], decimal=4)

    # Clean up our mess
    if remove_temporary:
        os.remove(kinematic_model_file_path)
        os.remove(c3d_file_path)


def main():
    # Create the model from user defined dimensions
    model_creation_from_static()

    # Create the model from a data file and markers as template
    model_creation_from_data()


if __name__ == "__main__":
    main()
