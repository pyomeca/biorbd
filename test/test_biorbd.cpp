#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"

static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
TEST(FileIO, OpenModel){
#ifdef MODULE_ACTUATORS
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
#else // MODULE_ACTUATORS
    EXPECT_THROW(biorbd::Model model(modelPathForGeneralTesting), std::runtime_error);
#endif // MODULE_ACTUATORS
}

TEST(GenericTests, mass){
    biorbd::Model model(modelPathForGeneralTesting);
    EXPECT_DOUBLE_EQ(model.mass(), 52.412120000000002);
}

static std::string modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
TEST(Constraint, loopConstraint){
    biorbd::Model model(modelPathForLoopConstraintTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model), QDot(model), QDDot_constrained(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque GeneralizedTorque(model);
    Q.setZero();
    QDot.setZero();
    QDDot_constrained.setZero();
    QDDot_expected << 0.35935119225397999,  -10.110263945851218,  -6.0044250056047249e-14, -27.780105329642453, 31.783042765424835, 44.636461505611521,
            0.67809701484785911, -6.251969045745929e-16, 5.9580819677934563e-15, 15.822263679783546, 2.4880587040480365e-15, -9.930136612989094e-15;
    GeneralizedTorque.setZero();

    biorbd::rigidbody::Contacts& cs(model.getConstraints_nonConst());
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(model, Q, QDot, GeneralizedTorque, cs, QDDot_constrained);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot_constrained[i], QDDot_expected[i], 1e-6);
}


