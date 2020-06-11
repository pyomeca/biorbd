#include "biorbd.h"

///
/// \brief main Get and print the generalized force of a model
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model
///     2. Position the model at a chosen position (Q), velocity (Qdot) and acceleration (Qddot)
///     3. Compute the generalized forces (tau) at this state (inverse dynamics)
///     4. Print them to the console
///
/// Please note that this example will work only with the Eigen backend
///
int main()
{
    // Load a predefined model
    biorbd::Model model("pyomecaman.bioMod");

    // Choose a position/velocity/acceleration to compute dynamics from
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
    Q.setZero();
    Qdot.setZero();
    Qddot.setZero();

    // Proceed with the inverse dynamics
    auto Tau = model.InverseDynamics(Q, Qdot, Qddot);

    // Print them to the console
    std::cout << Tau.transpose() << std::endl;

    return 0;
}
