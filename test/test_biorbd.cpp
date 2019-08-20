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
#ifdef MODULE_ACTUATORS
    biorbd::Model model(modelPathForGeneralTesting);
    EXPECT_DOUBLE_EQ(model.mass(), 52.412120000000002);
#endif
}

TEST(Dynamics, Forward){
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model), QDot(model), QDDot(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    QDDot_expected << 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Tau.setZero();

    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot[i], QDDot_expected[i], 1e-6);
}

static std::string modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
TEST(Dynamics, ForwardLoopConstraint){
    biorbd::Model model(modelPathForLoopConstraintTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model), QDot(model), QDDot_constrained(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Q.setZero();
    QDot.setZero();
    QDDot_constrained.setZero();
    QDDot_expected << 0.35935119225397999,  -10.110263945851218,  0, -27.780105329642453, 31.783042765424835,
            44.636461505611521, 0.67809701484785911, 0, 0, 15.822263679783546, 0, 0;
    Tau.setZero();

    biorbd::rigidbody::Contacts& cs(model.getConstraints_nonConst());
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(model, Q, QDot, Tau, cs, QDDot_constrained);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot_constrained[i], QDDot_expected[i], 1e-6);
}


