if __name__ == "__main__":
    from .utils import brbd_to_test
    from .test_model import test_wrapper_model
    from .test_segments import test_wrapper_segments
    from .test_markers import test_wrapper_markers
    from .test_frames import test_wrapper_frames
    from .test_dynamics import test_wrapper_dynamics
    from .test_external_forces import test_wrapper_external_forces
    from .test_muscles import test_muscles
    from .test_kalman_filter import test_wrapper_kalman_filter
    from .test_static_optimization import test_static_optimization

    for brbd in brbd_to_test:
        test_wrapper_model(brbd)
        test_wrapper_segments(brbd)
        test_wrapper_markers(brbd)
        test_wrapper_frames(brbd)
        test_wrapper_dynamics(brbd)
        test_wrapper_external_forces(brbd)
        test_muscles(brbd)
        test_wrapper_kalman_filter(brbd)  # This test is long
        test_static_optimization(brbd)
