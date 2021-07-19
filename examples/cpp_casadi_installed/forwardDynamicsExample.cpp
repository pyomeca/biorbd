#include "biorbd.h"

///
/// \brief main Get and print the generalized acceleration of a model
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model
///     2. Position the model at a chosen position (Q), velocity (Qdot)
///     3. Compute the generalized acceleration (Qddot) assuming a set of generalized forces (forward dynamics)
///     4. Print them to the console
///
/// Please note that this example will work only with the Eigen backend
///
int main()
{
    // Load a predefined model
    biorbd::Model model("pyomecaman.bioMod");

    // Choose a position/velocity/torque to compute dynamics from
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    std::vector<double> val(model.nbQ());
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);

    // Print them to the console
    std::cout << QDDot << std::endl;

    return 0;
}
