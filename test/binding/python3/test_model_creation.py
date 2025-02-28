import os
import pytest

import numpy as np


brbd_to_test = []
try:
    import biorbd
    from biorbd import model_creation
    
    brbd_to_test.append([biorbd, model_creation])
except ModuleNotFoundError:
    pass
try:
    import biorbd_casadi
    from biorbd_casadi import model_creation
    
    brbd_to_test.append([biorbd_casadi, model_creation])
except ModuleNotFoundError: 
    pass

if not brbd_to_test:
    raise ModuleNotFoundError("No biorbd or biorbd_casadi found")

import ezc3d


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_model_creation_from_static(brbd, remove_temporary: bool = True):
    """
    Produces a model from real data
    """
    brbd, bmc = brbd

    kinematic_model_file_path = "temporary.bioMod"

    # Create a model holder
    bio_model = bmc.BiomechanicalModelReal()

    # The trunk segment
    bio_model.segments["TRUNK"] = bmc.SegmentReal(
        name="TRUNK",
        translations=bmc.Translations.YZ,
        rotations=bmc.Rotations.X,
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, 0.53))),
    )
    bio_model.segments["TRUNK"].add_marker(bmc.MarkerReal(name="PELVIS", parent_name="TRUNK"))

    # The head segment
    bio_model.segments["HEAD"] = bmc.SegmentReal(
        name="HEAD",
        parent_name="TRUNK",
        segment_coordinate_system=bmc.SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, 0.53)
        ),
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, 0.24))),
    )
    bio_model.segments["HEAD"].add_marker(bmc.MarkerReal(name="BOTTOM_HEAD", parent_name="HEAD", position=(0, 0, 0)))
    bio_model.segments["HEAD"].add_marker(bmc.MarkerReal(name="TOP_HEAD", parent_name="HEAD", position=(0, 0, 0.24)))
    bio_model.segments["HEAD"].add_marker(bmc.MarkerReal(name="HEAD_Z", parent_name="HEAD", position=(0, 0, 0.24)))
    bio_model.segments["HEAD"].add_marker(bmc.MarkerReal(name="HEAD_XZ", parent_name="HEAD", position=(0.24, 0, 0.24)))

    # The arm segment
    bio_model.segments["UPPER_ARM"] = bmc.SegmentReal(
        name="UPPER_ARM",
        parent_name="TRUNK",
        segment_coordinate_system=bmc.SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, 0.53)
        ),
        rotations=bmc.Rotations.X,
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, -0.28))),
    )
    bio_model.segments["UPPER_ARM"].add_marker(bmc.MarkerReal(name="SHOULDER", parent_name="UPPER_ARM", position=(0, 0, 0)))
    bio_model.segments["UPPER_ARM"].add_marker(bmc.MarkerReal(name="SHOULDER_X", parent_name="UPPER_ARM", position=(1, 0, 0)))
    bio_model.segments["UPPER_ARM"].add_marker(bmc.MarkerReal(name="SHOULDER_XY", parent_name="UPPER_ARM", position=(1, 1, 0)))

    bio_model.segments["LOWER_ARM"] = bmc.SegmentReal(
        name="LOWER_ARM",
        parent_name="UPPER_ARM",
        segment_coordinate_system=bmc.SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.28)
        ),
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, -0.27))),
    )
    bio_model.segments["LOWER_ARM"].add_marker(bmc.MarkerReal(name="ELBOW", parent_name="LOWER_ARM", position=(0, 0, 0)))
    bio_model.segments["LOWER_ARM"].add_marker(bmc.MarkerReal(name="ELBOW_Y", parent_name="LOWER_ARM", position=(0, 1, 0)))
    bio_model.segments["LOWER_ARM"].add_marker(bmc.MarkerReal(name="ELBOW_XY", parent_name="LOWER_ARM", position=(1, 1, 0)))

    bio_model.segments["HAND"] = bmc.SegmentReal(
        name="HAND",
        parent_name="LOWER_ARM",
        segment_coordinate_system=bmc.SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.27)
        ),
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, -0.19))),
    )
    bio_model.segments["HAND"].add_marker(bmc.MarkerReal(name="WRIST", parent_name="HAND", position=(0, 0, 0)))
    bio_model.segments["HAND"].add_marker(bmc.MarkerReal(name="FINGER", parent_name="HAND", position=(0, 0, -0.19)))
    bio_model.segments["HAND"].add_marker(bmc.MarkerReal(name="HAND_Y", parent_name="HAND", position=(0, 1, 0)))
    bio_model.segments["HAND"].add_marker(bmc.MarkerReal(name="HAND_YZ", parent_name="HAND", position=(0, 1, 1)))

    # The thigh segment
    bio_model.segments["THIGH"] = bmc.SegmentReal(
        name="THIGH",
        parent_name="TRUNK",
        rotations=bmc.Rotations.X,
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, -0.42))),
    )
    bio_model.segments["THIGH"].add_marker(bmc.MarkerReal(name="THIGH_ORIGIN", parent_name="THIGH", position=(0, 0, 0)))
    bio_model.segments["THIGH"].add_marker(bmc.MarkerReal(name="THIGH_X", parent_name="THIGH", position=(1, 0, 0)))
    bio_model.segments["THIGH"].add_marker(bmc.MarkerReal(name="THIGH_Y", parent_name="THIGH", position=(0, 1, 0)))

    # The shank segment
    bio_model.segments["SHANK"] = bmc.SegmentReal(
        name="SHANK",
        parent_name="THIGH",
        segment_coordinate_system=bmc.SegmentCoordinateSystemReal.from_euler_and_translation(
            (0, 0, 0), "xyz", (0, 0, -0.42)
        ),
        rotations=bmc.Rotations.X,
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, -0.43))),
    )
    bio_model.segments["SHANK"].add_marker(bmc.MarkerReal(name="KNEE", parent_name="SHANK", position=(0, 0, 0)))
    bio_model.segments["SHANK"].add_marker(bmc.MarkerReal(name="KNEE_Z", parent_name="SHANK", position=(0, 0, 1)))
    bio_model.segments["SHANK"].add_marker(bmc.MarkerReal(name="KNEE_XZ", parent_name="SHANK", position=(1, 0, 1)))

    # The foot segment
    bio_model.segments["FOOT"] = bmc.SegmentReal(
        name="FOOT",
        parent_name="SHANK",
        segment_coordinate_system=bmc.SegmentCoordinateSystemReal.from_euler_and_translation(
            (-np.pi / 2, 0, 0), "xyz", (0, 0, -0.43)
        ),
        rotations=bmc.Rotations.X,
        mesh=bmc.MeshReal(((0, 0, 0), (0, 0, 0.25))),
    )
    bio_model.segments["FOOT"].add_marker(bmc.MarkerReal(name="ANKLE", parent_name="FOOT", position=(0, 0, 0)))
    bio_model.segments["FOOT"].add_marker(bmc.MarkerReal(name="TOE", parent_name="FOOT", position=(0, 0, 0.25)))
    bio_model.segments["FOOT"].add_marker(bmc.MarkerReal(name="ANKLE_Z", parent_name="FOOT", position=(0, 0, 1)))
    bio_model.segments["FOOT"].add_marker(bmc.MarkerReal(name="ANKLE_YZ", parent_name="FOOT", position=(0, 1, 1)))

    # Put the model together, print it and print it to a bioMod file
    bio_model.write(kinematic_model_file_path)

    model = brbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 25
    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", model.nbQ(), 1)
        value = np.array(brbd.to_casadi_func("markers", model.markers, q_sym)(np.zeros((model.nbQ(),))))[:, -3]
    else:        
        value = model.markers(np.zeros((model.nbQ(),)))[-3].to_array()
    np.testing.assert_almost_equal(value, [0, 0.25, -0.85], decimal=4)

    if remove_temporary:
        os.remove(kinematic_model_file_path)


def write_markers_to_c3d(save_path: str, brbd, model):
    q = np.zeros(model.nbQ())
    marker_names = tuple(name.to_string() for name in model.markerNames())
    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", model.nbQ(), 1)
        marker_positions = np.array(brbd.to_casadi_func("markers", model.markers, q_sym)(q))[:, :, np.newaxis]
    else:
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
    from de_leva import DeLevaTable
    
    kinematic_model_file_path = "temporary.bioMod"
    c3d_file_path = "temporary.c3d"
    test_model_creation_from_static(brbd, remove_temporary=False)
    
    brbd, bmc = brbd  # This must be done after the call to test_model_creation_from_static 

    # Prepare a fake model and a fake static from the previous test
    model = brbd.Model(kinematic_model_file_path)
    write_markers_to_c3d(c3d_file_path, brbd, model)
    os.remove(kinematic_model_file_path)

    # Fill the kinematic chain model
    model = bmc.BiomechanicalModel()
    de_leva = DeLevaTable(total_mass=100, sex="female", bmc=bmc)

    model.segments["TRUNK"] = bmc.Segment(
        name="TRUNK",
        translations=bmc.Translations.YZ,
        rotations=bmc.Rotations.X,
        inertia_parameters=de_leva["TRUNK"],
    )
    model.segments["TRUNK"].add_marker(bmc.Marker("PELVIS"))

    model.segments["HEAD"] = bmc.Segment(
        name="HEAD",
        parent_name="TRUNK",
        segment_coordinate_system=bmc.SegmentCoordinateSystem(
            "BOTTOM_HEAD",
            first_axis=bmc.Axis(name=bmc.Axis.Name.Z, start="BOTTOM_HEAD", end="HEAD_Z"),
            second_axis=bmc.Axis(name=bmc.Axis.Name.X, start="BOTTOM_HEAD", end="HEAD_XZ"),
            axis_to_keep=bmc.Axis.Name.Z,
        ),
        mesh=bmc.Mesh(("BOTTOM_HEAD", "TOP_HEAD", "HEAD_Z", "HEAD_XZ", "BOTTOM_HEAD")),
        inertia_parameters=de_leva["HEAD"],
    )
    model.segments["HEAD"].add_marker(bmc.Marker("BOTTOM_HEAD"))
    model.segments["HEAD"].add_marker(bmc.Marker("TOP_HEAD"))
    model.segments["HEAD"].add_marker(bmc.Marker("HEAD_Z"))
    model.segments["HEAD"].add_marker(bmc.Marker("HEAD_XZ"))

    model.segments["UPPER_ARM"] = bmc.Segment(
        name="UPPER_ARM",
        parent_name="TRUNK",
        rotations=bmc.Rotations.X,
        segment_coordinate_system=bmc.SegmentCoordinateSystem(
            origin="SHOULDER",
            first_axis=bmc.Axis(name=bmc.Axis.Name.X, start="SHOULDER", end="SHOULDER_X"),
            second_axis=bmc.Axis(name=bmc.Axis.Name.Y, start="SHOULDER", end="SHOULDER_XY"),
            axis_to_keep=bmc.Axis.Name.X,
        ),
        inertia_parameters=de_leva["UPPER_ARM"],
    )
    model.segments["UPPER_ARM"].add_marker(bmc.Marker("SHOULDER"))
    model.segments["UPPER_ARM"].add_marker(bmc.Marker("SHOULDER_X"))
    model.segments["UPPER_ARM"].add_marker(bmc.Marker("SHOULDER_XY"))

    model.segments["LOWER_ARM"] = bmc.Segment(
        name="LOWER_ARM",
        parent_name="UPPER_ARM",
        segment_coordinate_system=bmc.SegmentCoordinateSystem(
            origin="ELBOW",
            first_axis=bmc.Axis(name=bmc.Axis.Name.Y, start="ELBOW", end="ELBOW_Y"),
            second_axis=bmc.Axis(name=bmc.Axis.Name.X, start="ELBOW", end="ELBOW_XY"),
            axis_to_keep=bmc.Axis.Name.Y,
        ),
        inertia_parameters=de_leva["LOWER_ARM"],
    )
    model.segments["LOWER_ARM"].add_marker(bmc.Marker("ELBOW"))
    model.segments["LOWER_ARM"].add_marker(bmc.Marker("ELBOW_Y"))
    model.segments["LOWER_ARM"].add_marker(bmc.Marker("ELBOW_XY"))

    model.segments["HAND"] = bmc.Segment(
        name="HAND",
        parent_name="LOWER_ARM",
        segment_coordinate_system=bmc.SegmentCoordinateSystem(
            origin="WRIST",
            first_axis=bmc.Axis(name=bmc.Axis.Name.Y, start="WRIST", end="HAND_Y"),
            second_axis=bmc.Axis(name=bmc.Axis.Name.Z, start="WRIST", end="HAND_YZ"),
            axis_to_keep=bmc.Axis.Name.Y,
        ),
        inertia_parameters=de_leva["HAND"],
    )
    model.segments["HAND"].add_marker(bmc.Marker("WRIST"))
    model.segments["HAND"].add_marker(bmc.Marker("FINGER"))
    model.segments["HAND"].add_marker(bmc.Marker("HAND_Y"))
    model.segments["HAND"].add_marker(bmc.Marker("HAND_YZ"))

    model.segments["THIGH"] = bmc.Segment(
        name="THIGH",
        parent_name="TRUNK",
        rotations=bmc.Rotations.X,
        segment_coordinate_system=bmc.SegmentCoordinateSystem(
            origin="THIGH_ORIGIN",
            first_axis=bmc.Axis(name=bmc.Axis.Name.X, start="THIGH_ORIGIN", end="THIGH_X"),
            second_axis=bmc.Axis(name=bmc.Axis.Name.Y, start="THIGH_ORIGIN", end="THIGH_Y"),
            axis_to_keep=bmc.Axis.Name.X,
        ),
        inertia_parameters=de_leva["THIGH"],
    )
    model.segments["THIGH"].add_marker(bmc.Marker("THIGH_ORIGIN"))
    model.segments["THIGH"].add_marker(bmc.Marker("THIGH_X"))
    model.segments["THIGH"].add_marker(bmc.Marker("THIGH_Y"))

    model.segments["SHANK"] = bmc.Segment(
        name="SHANK",
        parent_name="THIGH",
        rotations=bmc.Rotations.X,
        segment_coordinate_system=bmc.SegmentCoordinateSystem(
            origin="KNEE",
            first_axis=bmc.Axis(name=bmc.Axis.Name.Z, start="KNEE", end="KNEE_Z"),
            second_axis=bmc.Axis(name=bmc.Axis.Name.X, start="KNEE", end="KNEE_XZ"),
            axis_to_keep=bmc.Axis.Name.Z,
        ),
        inertia_parameters=de_leva["SHANK"],
    )
    model.segments["SHANK"].add_marker(bmc.Marker("KNEE"))
    model.segments["SHANK"].add_marker(bmc.Marker("KNEE_Z"))
    model.segments["SHANK"].add_marker(bmc.Marker("KNEE_XZ"))

    model.segments["FOOT"] = bmc.Segment(
        name="FOOT",
        parent_name="SHANK",
        rotations=bmc.Rotations.X,
        segment_coordinate_system=bmc.SegmentCoordinateSystem(
            origin="ANKLE",
            first_axis=bmc.Axis(name=bmc.Axis.Name.Z, start="ANKLE", end="ANKLE_Z"),
            second_axis=bmc.Axis(name=bmc.Axis.Name.Y, start="ANKLE", end="ANKLE_YZ"),
            axis_to_keep=bmc.Axis.Name.Z,
        ),
        inertia_parameters=de_leva["FOOT"],
    )
    model.segments["FOOT"].add_marker(bmc.Marker("ANKLE"))
    model.segments["FOOT"].add_marker(bmc.Marker("TOE"))
    model.segments["FOOT"].add_marker(bmc.Marker("ANKLE_Z"))
    model.segments["FOOT"].add_marker(bmc.Marker("ANKLE_YZ"))

    # Put the model together, print it and print it to a bioMod file
    model.write(kinematic_model_file_path, bmc.C3dData(c3d_file_path))

    model = brbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 7
    assert model.nbSegment() == 8
    assert model.nbMarkers() == 25
    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", model.nbQ(), 1)
        value = np.array(brbd.to_casadi_func("markers", model.markers, q_sym)(np.zeros((model.nbQ(),))))[:, -3]
    else:
        value = model.markers(np.zeros((model.nbQ(),)))[-3].to_array()
    np.testing.assert_almost_equal(value, [0, 0.25, -0.85], decimal=4)
    
    if remove_temporary:
        os.remove(kinematic_model_file_path)
        os.remove(c3d_file_path)



@pytest.mark.parametrize("brbd", brbd_to_test)
def test_complex_model(brbd, remove_temporary: bool = True):
    brbd, bmc = brbd
    
    current_path_folder = os.path.dirname(os.path.realpath(__file__))
    mesh_path = f"{current_path_folder}/../../models/meshFiles/stl/pendulum.STL"
    
    kinematic_model_file_path = "temporary_complex.bioMod"

    # Create a model holder
    bio_model = bmc.BiomechanicalModel()

    # The ground segment
    bio_model.segments["GROUND"] = bmc.Segment(name="GROUND")

    # The pendulum segment
    bio_model.segments["PENDULUM"] = bmc.Segment(
        name="PENDULUM",
        translations=bmc.Translations.XYZ,
        rotations=bmc.Rotations.X,
        q_ranges=bmc.RangeOfMotion(range_type=bmc.Ranges.Q, min_bound=[-1, -1, -1, -np.pi], max_bound=[1, 1, 1, np.pi]),
        qdot_ranges=bmc.RangeOfMotion(range_type=bmc.Ranges.Qdot, min_bound=[-10, -10, -10, -np.pi*10], max_bound=[10, 10, 10, np.pi*10]),
        mesh_file=bmc.MeshFile(mesh_file_name=mesh_path,
                            mesh_color=np.array([0, 0, 1]),
                            scaling_function=lambda m: np.array([1, 1, 10]),
                            rotation_function=lambda m: np.array([np.pi/2, 0, 0]),
                            translation_function=lambda m: np.array([0.1, 0, 0])),
    )
    # The pendulum segment contact point
    bio_model.segments["PENDULUM"].add_contact(bmc.Contact(name="PENDULUM_CONTACT",
                                                        function=lambda m: np.array([0, 0, 0]),
                                                        parent_name="PENDULUM",
                                                        axis=bmc.Translations.XYZ))

    # The pendulum muscle group
    bio_model.muscle_groups["PENDULUM_MUSCLE_GROUP"] = bmc.MuscleGroup(name="PENDULUM_MUSCLE_GROUP",
                                                                    origin_parent_name="GROUND",
                                                                    insertion_parent_name="PENDULUM")

    # The pendulum muscle
    bio_model.muscles["PENDULUM_MUSCLE"] = bmc.Muscle("PENDULUM_MUSCLE",
                                                muscle_type=bmc.MuscleType.HILLTHELEN,
                                                state_type=bmc.MuscleStateType.DEGROOTE,
                                                muscle_group="PENDULUM_MUSCLE_GROUP",
                                                origin_position_function=lambda m: np.array([0, 0, 0]),
                                                insertion_position_function=lambda m: np.array([0, 0, 1]),
                                                optimal_length_function=lambda model, m: 0.1,
                                                maximal_force_function=lambda m: 100.0,
                                                tendon_slack_length_function=lambda model, m: 0.05,
                                                pennation_angle_function=lambda model, m: 0.05,
                                                maximal_excitation=1)
    bio_model.via_points["PENDULUM_MUSCLE"] = bmc.ViaPoint("PENDULUM_MUSCLE",
                                                        position_function=lambda m: np.array([0, 0, 0.5]),
                                                        parent_name="PENDULUM",
                                                        muscle_name="PENDULUM_MUSCLE",
                                                        muscle_group="PENDULUM_MUSCLE_GROUP",
                                                        )


    # Put the model together, print it and print it to a bioMod file
    bio_model.write(kinematic_model_file_path, {})

    model = brbd.Model(kinematic_model_file_path)
    assert model.nbQ() == 4
    assert model.nbSegment() == 2
    assert model.nbMarkers() == 0
    assert model.nbMuscles() == 1
    assert model.nbMuscleGroups() == 1
    assert model.nbContacts() == 3

    if remove_temporary:
        os.remove(kinematic_model_file_path)
