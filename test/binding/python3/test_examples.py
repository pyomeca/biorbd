import pytest

available_biorbd = []

try:
    import biorbd
    available_biorbd.append("biorbd")
except ImportError:
    pass

try:
    import biorbd_casadi
    available_biorbd.append("biorbd_casadi")
except ImportError:
    pass


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
    

def test_example_forward_dynamics():
    if "biorbd" not in available_biorbd:
        pytest.skip("This example is written for biorbd, but not for the biorbd_casadi")
    
    forward_dynamics = load_example_file("forwardDynamics.py")
    forward_dynamics.main()
    
    
def test_example_forward_dynamics_from_muscles():
    if "biorbd" not in available_biorbd:
        pytest.skip("This example is written for biorbd, but not for the biorbd_casadi")
        
    forward_dynamics = load_example_file("forwardDynamicsFromMuscles.py")
    forward_dynamics.main()
    
    
def test_example_forward_kinematics():
    if "biorbd" not in available_biorbd:
        pytest.skip("This example is written for biorbd, but not for the biorbd_casadi")
    
    forward_kinematics = load_example_file("forwardKinematics.py")
    forward_kinematics.main(show=False)
    

def test_example_inverse_dynamics():
    if "biorbd" not in available_biorbd:
        pytest.skip("This example is written for biorbd, but not for the biorbd_casadi")
    
    inverse_dynamics = load_example_file("inverseDynamics.py")
    inverse_dynamics.main()
    
    
def test_example_inverse_kinematics_kalman():
    if "biorbd" not in available_biorbd:
        pytest.skip("This example is written for biorbd, but not for the biorbd_casadi")
    
    inverse_dynamics_kalman = load_example_file("inverseKinematicsKalman.py")
    inverse_dynamics_kalman.main(show=False)
    
    
def test_example_model_creation():
    if "biorbd" not in available_biorbd:
        pytest.skip("This example is written for biorbd, but not for the biorbd_casadi")
    
    model_creation = load_example_file("modelCreation.py")
    model_creation.main()
    
    
def test_example_static_optimization():
    if "biorbd" not in available_biorbd:
        pytest.skip("This example is written for biorbd, but not for the biorbd_casadi")
    
    static_optimization = load_example_file("staticOptimization.py")
    static_optimization.main()
