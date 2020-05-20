#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>


#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"
#include "Actuators/ActuatorConstant.h"
#include "Actuators/ActuatorGauss3p.h"
#include "Actuators/ActuatorGauss6p.h"
#include "Actuators/ActuatorLinear.h"

static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
static std::string modelWithMissingActuator("models/withMissingActuator.bioMod");
static double requiredPrecision(1e-10);

TEST(FileIO, openModelWithActuators){
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
//    EXPECT_THROW(biorbd::Model model(modelWithMissingActuator), std::runtime_error);
}

TEST(ActuatorConstant, torqueMax){
    biorbd::actuator::ActuatorConstant const_torque_act(1, 150, 0);
    EXPECT_NEAR(const_torque_act.torqueMax(), 150, requiredPrecision);
}

TEST(ActuatorGauss3p, torqueMax){
    std::vector<int> QDot_sign {1, -1};
    unsigned int size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    biorbd::rigidbody::GeneralizedVelocity QDot(size);
    std::vector<double> torque_max_expected {0.084907296057452161, 1.8246053174014869};

    {
        unsigned int i = 0;
        for (unsigned int j = 0; j < size; ++j) {
            Q[j] = 1.0;
            QDot[j] = QDot_sign[i]*10;
        }
        biorbd::actuator::ActuatorGauss3p gauss3p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 0);
        EXPECT_NEAR(gauss3p_torque_act.torqueMax(Q, QDot), torque_max_expected[i], requiredPrecision);
    }
    {
        unsigned int i = 1;
        for (unsigned int j = 0; j < size; ++j) {
            Q[j] = 1.0;
            QDot[j] = QDot_sign[i]*10;
        }
        biorbd::actuator::ActuatorGauss3p gauss3p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 0);
        EXPECT_NEAR(gauss3p_torque_act.torqueMax(Q, QDot), torque_max_expected[i], requiredPrecision);
    }
}

TEST(ActuatorGauss6p, torqueMax){
    std::vector<int> QDot_sign {1, -1};
    unsigned int size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    biorbd::rigidbody::GeneralizedVelocity QDot(size);
    std::vector<double> torqueMaxExpected {10.101964984626497, 217.0849842477746};

    for (unsigned int i = 0; i < 2; ++i) {
        for (unsigned int j = 0; j < size; j++) {
            Q[j] = 1.0;
            QDot[j] = QDot_sign[i]*10;
        }
        biorbd::actuator::ActuatorGauss6p gauss6p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 4, 73, 73, 0);
        EXPECT_NEAR(gauss6p_torque_act.torqueMax(Q, QDot), torqueMaxExpected[i], requiredPrecision);
    }
}

TEST(ActuatorLinear, torqueMax){
    unsigned int size(1);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    Q[0] = 0.1;
    double torque_max_expected(30.729577951308233);
    biorbd::actuator::ActuatorLinear linear_torque_act(1, 25, 1, 0);
    CALL_BIORBD_FUNCTION_1ARG(val, linear_torque_act, torqueMax, Q)
    EXPECT_NEAR(val, torque_max_expected, requiredPrecision);
}

TEST(Actuators, jointTorqueFromActuators){
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i){
        val[i] = 1.1;
    }
    std::vector<double> val_act(model.nbGeneralizedTorque());
    for (size_t i=0; i<val_act.size(); ++i){
        val_act[i] = 0.5;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    DECLARE_VECTOR(actuatorActivations, val_act.size());
    FILL_VECTOR(actuatorActivations, val_act);

    CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
    for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
         EXPECT_NEAR(tau(i, 0), 150, requiredPrecision);
    }
}
