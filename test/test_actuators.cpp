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


static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
static double requiredPrecision(1e-10);

TEST(FileIO, OpenModelActuators){
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
}

TEST(Actuators, ActuatorConstant_TorqueMax){
    biorbd::actuator::ActuatorConstant positive_const_torque_act(1, 150, 0);
    //double torque_max_expected(150.0);
    // biorbd::rigidbody::GeneralizedTorque torque_max;
    EXPECT_NEAR(positive_const_torque_act.torqueMax(), 150, requiredPrecision);
    std::cout << "Test for positive direction passed" << std::endl;

    biorbd::actuator::ActuatorConstant negative_const_torque_act(1, 150, 0);
    EXPECT_NEAR(negative_const_torque_act.torqueMax(), -150, requiredPrecision);
    // Positive value of torqueMax when direction = -1 is normal ? Should not be -150 ?
    std::cout << "Test for negative direction passed" << std::endl;
}

TEST(Actuators, ActuatorGauss3p_TorqueMax){
    biorbd::actuator::ActuatorGauss3p positive_gauss3p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 0);
    unsigned int size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    biorbd::rigidbody::GeneralizedVelocity Qdot(size);
    for (unsigned int i = 0; i < size; i++) {
        Q[i] = 1.0;
        Qdot[i] = 10;
    }
    //double torque_max_expected(150.0);
    // biorbd::rigidbody::GeneralizedTorque torque_max;
    EXPECT_NEAR(positive_gauss3p_torque_act.torqueMax(Q, Qdot), 150, requiredPrecision);
    std::cout << "Test for positive direction passed" << std::endl;

    biorbd::actuator::ActuatorGauss3p negative_gauss3p_torque_act(-1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 0);
    EXPECT_NEAR(positive_gauss3p_torque_act.torqueMax(Q, Qdot), 150, requiredPrecision);
    std::cout << "Test for negative direction passed" << std::endl;
}

TEST(Actuators, ActuatorGauss6p_TorqueMax){
    biorbd::actuator::ActuatorGauss6p positive_gauss6p_torque_act(1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 4, 73, 73, 0);
    unsigned int size(5);
    biorbd::rigidbody::GeneralizedCoordinates Q(size);
    biorbd::rigidbody::GeneralizedVelocity Qdot(size);
    for (unsigned int i = 0; i < size; i++) {
        Q[i] = 1.0;
        Qdot[i] = 10;
    }
    //double torque_max_expected(150.0);
    // biorbd::rigidbody::GeneralizedTorque torque_max;
    EXPECT_NEAR(positive_gauss6p_torque_act.torqueMax(Q, Qdot), 150, requiredPrecision);
    std::cout << "Test for positive direction passed" << std::endl;

    biorbd::actuator::ActuatorGauss6p negative_gauss6p_torque_act(-1, 150, 25, 800, 324, 0.5, 28, 90, 29, 133, 4, 73, 73, 0);
    EXPECT_NEAR(positive_gauss6p_torque_act.torqueMax(Q, Qdot), 150, requiredPrecision);
    std::cout << "Test for negative direction passed" << std::endl;
}
