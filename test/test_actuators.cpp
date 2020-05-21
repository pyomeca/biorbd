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
static std::string modelPathWithMissingActuator("models/withMissingActuator.bioMod");
static std::string modelPathWithoutActuator("models/pyomecaman.bioMod");
static std::string modelPathWithAllActuators("models/withAllActuatorsTypes.bioMod");

static double requiredPrecision(1e-10);

TEST(FileIO, openModelWithActuators){
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithoutActuator));
    EXPECT_THROW(biorbd::Model model(modelPathWithMissingActuator), std::runtime_error);
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithAllActuators));

}

TEST(ActuatorConstant, torqueMax){
    biorbd::actuator::ActuatorConstant const_torque_act(1, 150, 0);
    SCALAR_TO_DOUBLE(torqueMaxVal, const_torque_act.torqueMax())
    EXPECT_NEAR(torqueMaxVal, 150, requiredPrecision);
}

TEST(ActuatorGauss3p, torqueMax){
    biorbd::actuator::ActuatorGauss3p gauss3p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 0);
    size_t size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    std::vector<double> Q_val = {1.0, 1.0, 1.0, 1.0, 1.0};
    FILL_VECTOR(Q, Q_val);
    biorbd::rigidbody::GeneralizedVelocity QDot(size);
    {
        std::vector<double> QDot_val = {10, 10, 10, 10, 10};
        double torqueMaxExpected(0.084907296057452161);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss3p_torque_act, torqueMax, Q, QDot);
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
    }
    {
        std::vector<double> QDot_val = {-10, -10, -10, -10, -10};
        double torqueMaxExpected(1.8246053174014869);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss3p_torque_act, torqueMax, Q, QDot);
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
    }
}

TEST(ActuatorGauss6p, torqueMax){
    biorbd::actuator::ActuatorGauss6p gauss6p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 4, 73, 73, 0);
    size_t size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    std::vector<double> Q_val = {1.0, 1.0, 1.0, 1.0, 1.0};
    FILL_VECTOR(Q, Q_val);
    biorbd::rigidbody::GeneralizedVelocity QDot(size);
    {
        std::vector<double> QDot_val = {10, 10, 10, 10, 10};
        double torqueMaxExpected(10.101964984626497);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss6p_torque_act, torqueMax, Q, QDot);
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
    }
    {
        std::vector<double> QDot_val = {-10, -10, -10, -10, -10};
        double torqueMaxExpected(217.0849842477746);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss6p_torque_act, torqueMax, Q, QDot);
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
    }
} 

TEST(ActuatorLinear, torqueMax){
    unsigned int size(1);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    std::vector<double> val = {1};
    FILL_VECTOR(Q, val);
    double torqueMaxExpected(82.295779513082323);
    biorbd::actuator::ActuatorLinear linear_torque_act(1, 25, 1, 0);
    CALL_BIORBD_FUNCTION_1ARG(torqueMaxVal, linear_torque_act, torqueMax, Q);
    EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
}

TEST(Actuators, NbActuators){
    {
        biorbd::Model model(modelPathForGeneralTesting);
        size_t val(model.nbActuators());
        EXPECT_NEAR(val, 13, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathWithoutActuator);
        size_t val(model.nbActuators());
        EXPECT_NEAR(val, 0, requiredPrecision);
    }
}

TEST(Actuators, jointTorqueFromActuators){
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_VECTOR(actuatorActivations, model.nbGeneralizedTorque());

    std::vector<double> Q_val(model.nbQ());
    for (size_t i=0; i<Q_val.size(); ++i){
        Q_val[i] = 1.1;
    }
    std::vector<double> QDot_val(model.nbQdot());
    for (size_t i=0; i<QDot_val.size(); ++i){
        QDot_val[i] = 1.1;
    }
    std::vector<double> act_val(model.nbGeneralizedTorque());
    for (size_t i=0; i<act_val.size(); ++i){
        act_val[i] = 0.5;
    }

    FILL_VECTOR(Q, Q_val);
    FILL_VECTOR(QDot, QDot_val);
    FILL_VECTOR(actuatorActivations, act_val);

    CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
    EXPECT_NEAR(tau.size(), 13, requiredPrecision);
    for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
         EXPECT_NEAR(tau(i, 0), 150, requiredPrecision);
    }
    CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q, QDot);
    EXPECT_NEAR(torqueMax.size(), 13, requiredPrecision);
    for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
         EXPECT_NEAR(torqueMax(i, 0), 150, requiredPrecision);
    }
}

TEST(Actuators, jointTorqueFromAllTypesOfActuators){
    biorbd::Model model(modelPathWithAllActuators);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_VECTOR(actuatorActivations, model.nbGeneralizedTorque());
    std::vector<double> Q_val(model.nbQ());
    for (size_t i=0; i<Q_val.size(); ++i){
        Q_val[i] = 1.1;
    }
    FILL_VECTOR(Q, Q_val);
    {
        std::vector<double> QDot_val(model.nbQdot());
        for (size_t i=0; i<QDot_val.size(); ++i){
            QDot_val[i] = 1.1;
        }
        FILL_VECTOR(QDot, QDot_val);

        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i){
                act_val[i] = 0.5;
            }
            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
            std::vector<double> torqueMaxExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
            EXPECT_NEAR(tau.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(tau(i, 0), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q, QDot);
            EXPECT_NEAR(torqueMax.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(torqueMax(i, 0), torqueMaxExpected[i], requiredPrecision);
            }
        }
        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i){
                act_val[i] = -0.5;
            }

            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
            std::vector<double> torqueMaxExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
            EXPECT_NEAR(tau.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(tau(i, 0), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q, QDot);
            EXPECT_NEAR(torqueMax.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(torqueMax(i, 0), torqueMaxExpected[i], requiredPrecision);
            }
        }

    }
    {
        std::vector<double> QDot_val(model.nbQdot());
        for (size_t i=0; i<QDot_val.size(); ++i){
            QDot_val[i] = -1.1;
        }
        FILL_VECTOR(QDot, QDot_val);
        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i){
                act_val[i] = 0.5;
            }

            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
            std::vector<double> torqueMaxExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
            EXPECT_NEAR(tau.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(tau(i, 0), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q, QDot);
            EXPECT_NEAR(torqueMax.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(torqueMax(i, 0), torqueMaxExpected[i], requiredPrecision);
            }
        }
        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i){
                act_val[i] = -0.5;
            }
            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
            std::vector<double> torqueMaxExpected = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
            EXPECT_NEAR(tau.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(tau(i, 0), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q, QDot);
            EXPECT_NEAR(torqueMax.size(), 13, requiredPrecision);
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i){
                 EXPECT_NEAR(torqueMax(i, 0), torqueMaxExpected[i], requiredPrecision);
            }
        }
    }
}

