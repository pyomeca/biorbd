#include "biorbd.h"

///
/// \brief main Get and print the generalized acceleration of a model
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model
///     2. Position the model at a chosen position (Q), velocity (Qdot)
///     3. Compute the generalized acceleration (Qddot) assuming a set of
///     generalized forces (forward dynamics)
///     4. Print them to the console
///
/// Please note that this example will work only with the Eigen backend
///

using namespace BIORBD_NAMESPACE;

int main() {
  // Load a predefined model
  Model model("pyomecaman.bioMod");

  // Choose a position/velocity/torque to compute dynamics from
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  Q.setZero();
  Qdot.setZero();
  Tau.setZero();

  // Proceed with the forward dynamics
  auto Qddot = model.ForwardDynamics(Q, Qdot, Tau);

  // Print them to the console
  std::cout << Qddot.transpose() << std::endl;

  return 0;
}
