#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>


#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"
#include "InternalForces/all.h"
#include "InternalForces/PassiveTorques/all.h"

using namespace BIORBD_NAMESPACE;

static std::string modelPathForGeneralTesting("models/arm26_WithPassiveTorques.bioMod");
static std::string modelPathOnePassiveTorque("models/arm26_WithOnePassiveTorques.bioMod");
static std::string modelPathWithoutPassiveTorques("models/arm26.bioMod");


static double requiredPrecision(1e-10);

TEST(FileIO, openModelWithPassiveTorques)
{
    EXPECT_NO_THROW(Model model(modelPathForGeneralTesting));
    EXPECT_NO_THROW(Model model(modelPathWithoutPassiveTorques));
    EXPECT_NO_THROW(Model model(modelPathOnePassiveTorque));
}

TEST(PassiveTorqueConstant, passiveTorque)
{
    internal_forces::passive_torques::PassiveTorqueConstant const_torque_act(150, 0);
    SCALAR_TO_DOUBLE(torqueConstantVal, const_torque_act.passiveTorque())
    EXPECT_NEAR(torqueConstantVal, 150, requiredPrecision);
}

TEST(PassiveTorqueLinear, passiveTorque)
{
    // A model is loaded so Q can be > 0 in size, it is not used otherwise
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);

    std::vector<double> val = {1.1, 1.1, 1.1};
    FILL_VECTOR(Q, val);
    double torqueLinearExpected(3.100000000000000);
    internal_forces::passive_torques::PassiveTorqueLinear linear_torque_act(2, 1, 1);
    CALL_BIORBD_FUNCTION_1ARG(torqueLinearVal, linear_torque_act, passiveTorque, Q);
#ifdef BIORBD_USE_CASADI_MATH
    EXPECT_NEAR(static_cast<double>(torqueLinearVal(0, 0)), torqueLinearExpected, requiredPrecision);
#else
    EXPECT_NEAR(torqueLinearVal, torqueLinearExpected, requiredPrecision);
#endif
}

TEST(PassiveTorqueExponential, passiveTorque)
{
    // A model is loaded so Q can be > 0 in size, it is not used otherwise
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_COORDINATES(Qdot, model);

    std::vector<double> val = {1.1, 1.1, 1.1};
    FILL_VECTOR(Q, val);
    FILL_VECTOR(Qdot, val);
    double torqueExponentialExpected(27.237093441022679);
    internal_forces::passive_torques::PassiveTorqueExponential exponential_torque_act(2, 2, 5, 5, 1, 8, 5, 2, 1, 2, 2);
    CALL_BIORBD_FUNCTION_2ARGS(torqueExponentialVal, exponential_torque_act, passiveTorque, Q, Qdot);
#ifdef BIORBD_USE_CASADI_MATH
    EXPECT_NEAR(static_cast<double>(torqueExponentialVal(0, 0)), torqueExponentialExpected, requiredPrecision);
#else
    EXPECT_NEAR(torqueExponentialVal, torqueExponentialExpected, requiredPrecision);
#endif
}

TEST(PassiveTorques, NbPassiveTorques)
{
    {
        Model model(modelPathForGeneralTesting);
        size_t val(model.nbPassiveTorques());
        EXPECT_EQ(val, 3);
    }
    {
        Model model(modelPathWithoutPassiveTorques);
        size_t val(model.nbPassiveTorques());
        EXPECT_EQ(val, 0);
    }
    {
        Model model(modelPathOnePassiveTorque);
        size_t val(model.nbPassiveTorques());
        EXPECT_EQ(val, 1);
    }
}

TEST(PassiveTorques, jointTorqueFromAllTypesOfPassiveTorque)
{
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(Qdot, model);
    std::vector<double> Q_val(model.nbQ());
    for (size_t i=0; i<Q_val.size(); ++i) {
        Q_val[i] = 1.1;
    }
    FILL_VECTOR(Q, Q_val);

    std::vector<double> Qdot_val(model.nbQdot());
    for (size_t i=0; i<Qdot_val.size(); ++i) {
        Qdot_val[i] = 1.1;
    }
    FILL_VECTOR(Qdot, Qdot_val);

    std::vector<double> torqueExpected = {3.100000000000000, -22.237006567213825, 5};

    CALL_BIORBD_FUNCTION_2ARGS(tau, model, passiveJointTorque, Q, Qdot);
#ifdef BIORBD_USE_CASADI_MATH
    EXPECT_NEAR(tau.size().first, 3, requiredPrecision);
#else
    EXPECT_EQ(tau.size(), 3);
#endif
    for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
        EXPECT_NEAR(static_cast<double>(tau(i, 0)), torqueExpected[i], requiredPrecision);
    }
}

TEST(PassiveTorques, onlyOnePassiveTorque)
{
    Model model(modelPathOnePassiveTorque);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(Qdot, model);
    std::vector<double> Q_val(model.nbQ());
    for (size_t i=0; i<Q_val.size(); ++i) {
        Q_val[i] = 1.1;
    }
    FILL_VECTOR(Q, Q_val);

    std::vector<double> Qdot_val(model.nbQdot());
    for (size_t i=0; i<Qdot_val.size(); ++i) {
        Qdot_val[i] = 1.1;
    }
    FILL_VECTOR(Qdot, Qdot_val);

    std::vector<double> torqueExpected = {3.100000000000000, 0};
     
    CALL_BIORBD_FUNCTION_2ARGS(tau, model, passiveJointTorque, Q, Qdot);
#ifdef BIORBD_USE_CASADI_MATH
    EXPECT_NEAR(tau.size().first, 2, requiredPrecision);
#else
    EXPECT_EQ(tau.size(), 2);
#endif
    for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
        EXPECT_NEAR(static_cast<double>(tau(i, 0)), torqueExpected[i], requiredPrecision);
    }
}
