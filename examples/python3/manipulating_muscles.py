from pathlib import Path

import biorbd

#
# This examples shows how to get and manipulate muscles using the biorbd Python bindings.
# Please note that this example will work only with the Eigen backend
#


def main():
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Biorbd(f"{current_file_dir}/../arm26.bioMod")
    muscles = model.muscles
    nmus = len(muscles)

    # Since manipulating muscles can be computationally intensive, we recommend to
    # precompute positions and velocities of the model
    q = [0.1] * model.nb_q
    qdot = [0.1] * model.nb_qdot
    muscles.update_geometry(q=q, qdot=qdot)
    # From that point on, it is possible (and recommended) to avoid sending any q or qdot to any muscle function.
    # Otherwise, the geometry will be updated again, which is not necessary and can be computationally intensive

    # Set the activations of the muscles
    muscles.activations = [0.5] * nmus

    # We can now compute the muscles forces
    forces = muscles.forces()
    print(f"Muscle forces: {forces}")

    # We can also compute the muscle length jacobian
    jacobian = muscles.length_jacobian()
    print(f"Muscle length jacobian: {jacobian}")

    # The joint torques is computed as -J^T * F
    tau = -jacobian.T @ forces
    print(f"Joint torques: {tau}")

    # The shortcut to the same joint torques is
    tau = muscles.joint_torque()
    print(f"Joint torques (shortcut): {tau}")

    # We can also print and change the properties of a specific muscle
    muscle = muscles[0]
    print(f"The name is:{muscle.name}")
    print(f"The optimal length is:{muscle.optimal_length}")
    print(f"The maximal isometric force is:{muscle.maximal_isometric_force}")
    print(f"The pcsa is:{muscle.pcsa}")
    print(f"The tendon slack length is:{muscle.tendon_slack_length}")
    print(f"The pennation angle is:{muscle.pennation_angle}")
    print(f"The maximal contraction velocity is:{muscle.maximal_contraction_velocity}")
    print(f"The force at that moment is:{muscle.force}")
    print(f"------")

    # Now change the values
    muscle.optimal_length = 0.2
    muscle.maximal_isometric_force = 1000.0
    muscle.pcsa = 0.02
    muscle.tendon_slack_length = 0.3
    muscle.pennation_angle = 0.1
    muscle.maximal_contraction_velocity = 15.0

    # And reprint them
    print(f"The name is:{muscle.name}")
    print(f"The optimal length is:{muscle.optimal_length}")
    print(f"The maximal isometric force is:{muscle.maximal_isometric_force}")
    print(f"The pcsa is:{muscle.pcsa}")
    print(f"The tendon slack length is:{muscle.tendon_slack_length}")
    print(f"The pennation angle is:{muscle.pennation_angle}")
    print(f"The maximal contraction velocity is:{muscle.maximal_contraction_velocity}")

    # Note: the muscle force will not properly propagate until the geometry is updated
    print(f"The force at that moment is:{muscle.force}")
    muscles.update_geometry(q=q, qdot=qdot)
    print(f"The force after geometry update is:{muscle.force}")


if __name__ == "__main__":
    main()
