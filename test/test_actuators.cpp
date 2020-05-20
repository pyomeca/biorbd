#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>


#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Utils/Range.h"
#include "Utils/SpatialVector.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/IMU.h"
#include "Actuators/ActuatorConstant.h"
#include "Actuators/ActuatorGauss3p.h"
#include "Actuators/ActuatorGauss6p.h"
#include "Actuators/ActuatorLinear.h"

static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
static double requiredPrecision(1e-10);

TEST(FileIO, OpenModelActuators){
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
}

//Tests for TorqueMax:

TEST(Actuators, ActuatorConstant_TorqueMax){
    biorbd::actuator::ActuatorConstant const_torque_act(1, 150, 0);
    EXPECT_NEAR(const_torque_act.torqueMax(), 150, requiredPrecision);
}

TEST(Actuators, ActuatorGauss3p_TorqueMax){
    std::vector<int> QDot_sign {1, -1};
    unsigned int size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    biorbd::rigidbody::GeneralizedVelocity QDot(size);
    std::vector<double> torque_max_expected {0.084907296057452161, 1.8246053174014869};

    for (unsigned int i = 0; i < 2; i++) {
        for (unsigned int j = 0; j < size; j++) {
            Q[j] = 1.0;
            QDot[j] = QDot_sign[i]*10;
        }
        biorbd::actuator::ActuatorGauss3p gauss3p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 0);
        EXPECT_NEAR(gauss3p_torque_act.torqueMax(Q, QDot), torque_max_expected[i], requiredPrecision);
    }
}

TEST(Actuators, ActuatorGauss6p_TorqueMax){
    std::vector<int> QDot_sign {1, -1};
    unsigned int size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    biorbd::rigidbody::GeneralizedVelocity QDot(size);
    std::vector<double> torque_max_expected {10.101964984626497, 217.0849842477746};

    for (unsigned int i = 0; i < 2; i++) {
        for (unsigned int j = 0; j < size; j++) {
            Q[j] = 1.0;
            QDot[j] = QDot_sign[i]*10;
        }
        biorbd::actuator::ActuatorGauss6p gauss6p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 4, 73, 73, 0);
        EXPECT_NEAR(gauss6p_torque_act.torqueMax(Q, QDot), torque_max_expected[i], requiredPrecision);
    }
}

TEST(Actuators, ActuatorLinear_TorqueMax){
    unsigned int size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    double torque_max_expected(25);
    biorbd::actuator::ActuatorLinear linear_torque_act(1, 25, 1, 0);
    // Whatever the value of the slope, linear_torque_act.torqueMax returns 25, it seems normal to me as it's linear, though (?)
    EXPECT_NEAR(linear_torque_act.torqueMax(Q), torque_max_expected, requiredPrecision);
}

//Tests for torque method:

TEST(Actuators, ActuatorConstant_Torque){

    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); i++){
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    biorbd::actuator::ActuatorConstant const_torque_act(1, 150, 0);
    //biorbd::utils::Vector vector_torque_act(1);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    //Tau = model.torque(vector_torque_act, Q, QDot);
    Tau = model.torque(const_torque_act, Q, QDot);
    EXPECT_NEAR(Tau, 150, requiredPrecision);
}

TEST(Actuators, ActuatorGauss3p_Torque){

    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); i++){
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    biorbd::actuator::ActuatorGauss3p gauss3p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 0);
    //biorbd::utils::Vector vector_torque_act(1);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Tau = model.torque(gauss3p_torque_act, Q, QDot);
    EXPECT_NEAR(Tau, 150, requiredPrecision);
}

TEST(Actuators, ActuatorGauss6p_Torque){

    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); i++){
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    biorbd::actuator::ActuatorGauss6p gauss6p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 4, 73, 73, 0);
    //biorbd::utils::Vector vector_torque_act(1);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Tau = model.torque(gauss6p_torque_act, Q, QDot);
    EXPECT_NEAR(Tau, 150, requiredPrecision);
}

TEST(Actuators, ActuatorLinear_Torque){

    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); i++){
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    biorbd::actuator::ActuatorLinear linear_torque_act(1, 25, 1, 0);
    //biorbd::utils::Vector vector_torque_act(1);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Tau = model.torque(linear_torque_act, Q, QDot);
    EXPECT_NEAR(Tau, 150, requiredPrecision);
}
