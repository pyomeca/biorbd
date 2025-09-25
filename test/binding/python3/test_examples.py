import pytest

brbd_to_test = []

try:
    import biorbd

    brbd_to_test.append(biorbd)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd: {e}")
    pass

try:
    import biorbd_casadi

    brbd_to_test.append(biorbd_casadi)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd_casadi: {e}")
    pass

if not brbd_to_test:
    raise RuntimeError("No biorbd version could be imported")


def load_example_file(file_name: str):
    # Import the file as a module from the path
    import os
    from pathlib import Path
    from importlib.util import spec_from_file_location, module_from_spec
    import sys

    # Get the path to the examples
    if os.environ.get("CI_MAIN_EXAMPLES_FOLDER"):
        examples_path = Path(os.environ["CI_MAIN_EXAMPLES_FOLDER"]) / "python3"
    else:
        examples_path = Path(__file__).parent.parent.parent.parent / "examples" / "python3"

    # Import the example
    sys.path.append(str(examples_path))
    spec = spec_from_file_location("example", examples_path / file_name)
    example = module_from_spec(spec)
    spec.loader.exec_module(example)

    sys.path.pop()

    return example


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_example_forward_dynamics(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip forward_dynamics example for biorbd_casadi")

    forward_dynamics = load_example_file("forward_dynamics.py")
    forward_dynamics.main()


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_example_forward_dynamics_from_muscles(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip forward_dynamics_from_muscles example for biorbd_casadi")

    forward_dynamics = load_example_file("forward_dynamics_from_muscles.py")
    forward_dynamics.main()


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_example_forward_kinematics(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip forward_kinematics example for biorbd_casadi")

    forward_kinematics = load_example_file("forward_kinematics.py")
    forward_kinematics.main(show=False)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_example_inverse_dynamics(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse_dynamics example for biorbd_casadi")

    inverse_dynamics = load_example_file("inverse_dynamics.py")
    inverse_dynamics.main()


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_example_inverse_kinematics_kalman(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse_kinematics_kalman example for biorbd_casadi")

    inverse_dynamics_kalman = load_example_file("inverse_kinematics_kalman.py")
    inverse_dynamics_kalman.main(show=False)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_example_static_optimization(brbd):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip static_optimization example for biorbd_casadi")

    static_optimization = load_example_file("static_optimization.py")
    static_optimization.main()


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_example_forward_dynamics(brbd)
        test_example_forward_dynamics_from_muscles(brbd)
        test_example_forward_kinematics(brbd)
        test_example_inverse_dynamics(brbd)
        test_example_inverse_kinematics_kalman(brbd)
        # test_example_static_optimization(brbd)
