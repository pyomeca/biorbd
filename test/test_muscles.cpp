#include <iostream>
#include <gtest/gtest.h>

#include <rbdl/Dynamics.h>
#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#ifdef MODULE_MUSCLES
#include "Muscles/Muscle.h"
#include "Muscles/Geometry.h"
#include "Muscles/Caracteristics.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/HillTypeThelenFatigable.h"
#include "Muscles/FatigueDynamicStateXia.h"
#include "Muscles/StateDynamics.h"

static std::string modelPathForMuscleForce("models/arm26.bioMod");
TEST(MuscleForce, torqueFromMuscles)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::rigidbody::GeneralizedCoordinates Q(model), QDot(model), QDDot(model), QDDotExpected(model);
    Q.setOnes()/10;
    QDot.setOnes()/10;
    std::vector<biorbd::muscles::StateDynamics> states;
    for (unsigned int i=0; i<model.nbMuscleTotal(); ++i)
        states.push_back(biorbd::muscles::StateDynamics(0, 0.2));
    QDDotExpected << -0.046239425378874474, 0.12579777534720743;

    biorbd::rigidbody::GeneralizedTorque Tau(model.muscularJointTorque(states, true, &Q, &QDot));
    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);

    for (unsigned int i=0; i<QDDot.size(); ++i)
        EXPECT_DOUBLE_EQ(QDDot[i], QDDotExpected[i]);

}

static std::string modelPathForMuscleJacobian("models/arm26.bioMod");
static unsigned int muscleGroupForMuscleJacobian(1);
static unsigned int muscleForMuscleJacobian(1);
TEST(MuscleJacobian, jacobian){
    biorbd::Model model(modelPathForMuscleJacobian);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    Q.setZero();

    // Force computation of geometry
    std::shared_ptr<biorbd::muscles::Muscle> muscle(model.muscleGroup(muscleForMuscleJacobian).muscle_nonConst(muscleGroupForMuscleJacobian));
    EXPECT_THROW(muscle->position().jacobian(), std::runtime_error);
    model.updateMuscles(Q, true);

    unsigned int nRows(3 * (muscle->pathChanger().nbObjects() + 2));
    biorbd::utils::Matrix jacoRef(nRows, model.nbQ());
    // Here we provide emperical values that we have confidence in (TODO finite differencing could be better)
    jacoRef(0, 0) = 0.136691;    jacoRef(0, 1) = 0;
    jacoRef(1, 0) = -0.00889905; jacoRef(1, 1) = 0;
    jacoRef(2, 0) = 0.00808536;  jacoRef(2, 1) = 0;
    jacoRef(3, 0) = 0.151102;    jacoRef(3, 1) = 0;
    jacoRef(4, 0) = -0.0266009;  jacoRef(4, 1) = 0;
    jacoRef(5, 0) = 0.00897639;  jacoRef(5, 1) = 0;
    jacoRef(6, 0) = 0.225948;    jacoRef(6, 1) = 0;
    jacoRef(7, 0) = -0.0325014;  jacoRef(7, 1) = 0;
    jacoRef(8, 0) = 0.013406;    jacoRef(8, 1) = 0;
    jacoRef(9, 0) = 0.267077;    jacoRef(9, 1) = 0;
    jacoRef(10, 0) = -0.0181112; jacoRef(10, 1) = 0;
    jacoRef(11, 0) = 0.0157994;  jacoRef(11, 1) = 0;
    jacoRef(12, 0) = 0.279423;   jacoRef(12, 1) = -0.0104688;
    jacoRef(13, 0) = -0.0165429; jacoRef(13, 1) = -0.02182;
    jacoRef(14, 0) = 0.0165243;  jacoRef(14, 1) = 0.00131826;

    // Compare with computed values
    EXPECT_LT( (muscle->position().jacobian() - jacoRef).squaredNorm(), 1e-6);
}
TEST(MuscleJacobian, jacobianLength){
    biorbd::Model model(modelPathForMuscleJacobian);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    Q.setZero();
    model.updateMuscles(Q, true);

    unsigned int nRows(model.nbMuscleTotal());
    biorbd::utils::Matrix jacoRef(nRows, model.nbQ());
    // Here we provide emperical values that we have confidence in (TODO finite differencing could be better)
    jacoRef(0, 0) = 0.0374639;      jacoRef(0, 1) = 0.0200953;
    jacoRef(1, 0) = -0.0166515;     jacoRef(1, 1) = -0.00937993;
    jacoRef(2, 0) = -0.00743218;    jacoRef(2, 1) = -0.00937993;
    jacoRef(3, 0) = -1.11881e-17;   jacoRef(3, 1) = 0.0200953;
    jacoRef(4, 0) = -2.95716e-17;   jacoRef(4, 1) = 0.0200953;
    jacoRef(5, 0) = -1.05977e-18;   jacoRef(5, 1) = 0.00262888;

    // Compare with computed values
    EXPECT_LT( (model.musclesLengthJacobian(Q) - jacoRef).squaredNorm(), 1e-6);
}

static std::string modelPathForXiaDerivativeTest("models/arm26.bioMod");
static unsigned int muscleGroupForXiaDerivativeTest(0);
static unsigned int muscleForXiaDerivativeTest(0);
static double activationEmgForXiaDerivativeTest(1.0);
static double currentActiveFibersForXiaDerivativeTest(0.9);
static double currentFatiguedFibersForXiaDerivativeTest(0.0);
static double currentRestingFibersForXiaDerivativeTest(0.1);
static double expectedActivationDotForXiaDerivativeTest(0.991);
static double expectedFatiguedDotForXiaDerivativeTest(0.009);
static double expectedRestingDotForXiaDerivativeTest(-1.0);
static double positiveFibersQuantityForFatigueXiaSetStateLimitsTest(1.0);
static double negativeFibersQuantityForFatigueXiaSetStateLimitsTest(-1.0);
static double excessiveFibersQuantityForFatigueXiaSetStateLimitsTest(1.5);
TEST(MuscleFatigue, FatigueXiaDerivativeViaPointers){
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedCoordinates QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        std::shared_ptr<biorbd::muscles::HillTypeThelenFatigable> muscle(
                    std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(
                        model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<biorbd::muscles::FatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works
        // Sanity check for the fatigue model
        EXPECT_STREQ(fatigueModel->getType().c_str(), "Xia");

        // Initial value sanity check
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), 0);

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0, activationEmgForXiaDerivativeTest); // Set target
        fatigueModel->setState(currentActiveFibersForXiaDerivativeTest,
                               currentFatiguedFibersForXiaDerivativeTest,
                               currentRestingFibersForXiaDerivativeTest);
        fatigueModel->timeDerivativeState(emg, model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)->caract());

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), expectedActivationDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), expectedRestingDotForXiaDerivativeTest);
    }

    // Values should be changed in the model itself
    {
        std::shared_ptr<biorbd::muscles::HillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<biorbd::muscles::FatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), expectedActivationDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), expectedRestingDotForXiaDerivativeTest);
    }
}

TEST(MuscleFatigue, FatigueXiaDerivativeViaInterface){
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedCoordinates QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        std::shared_ptr<biorbd::muscles::HillTypeThelenFatigable> muscle(
                    std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(
                        model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0, activationEmgForXiaDerivativeTest); // Set target
        muscle->fatigueState(currentActiveFibersForXiaDerivativeTest,
                             currentFatiguedFibersForXiaDerivativeTest,
                             currentRestingFibersForXiaDerivativeTest);
        muscle->computeTimeDerivativeState(emg);

    }

    // Values should be changed in the model itself
    {
        std::shared_ptr<biorbd::muscles::HillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(model.muscleGroup(0).muscle(0)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<biorbd::muscles::FatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), expectedActivationDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), expectedRestingDotForXiaDerivativeTest);
    }
}

TEST(MuscleFatigue, FatigueXiaDerivativeViaCopy){
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedCoordinates QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillTypeThelenFatigable muscle(
                    model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
        // Sanity check for the fatigue model
        EXPECT_STREQ(fatigueModel.getType().c_str(), "Xia");

        // Initial value sanity check
        EXPECT_DOUBLE_EQ(fatigueModel.activeFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel.fatiguedFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel.restingFibersDot(), 0);

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0, activationEmgForXiaDerivativeTest); // Set target
        fatigueModel.setState(currentActiveFibersForXiaDerivativeTest,
                              currentFatiguedFibersForXiaDerivativeTest,
                              currentRestingFibersForXiaDerivativeTest);
        fatigueModel.timeDerivativeState(emg, model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)->caract());

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel.activeFibersDot(), expectedActivationDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel.fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel.restingFibersDot(), expectedRestingDotForXiaDerivativeTest);
    }

    // Values should not be changed in the model itself
    {
        std::shared_ptr<biorbd::muscles::HillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<biorbd::muscles::FatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), 0);
    }
}

TEST(MuscleFatigue, FatigueXiaSetStateLimitsTest){
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::muscles::HillTypeThelenFatigable muscle(
                model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
    biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
    // Sanity check for the fatigue model
    EXPECT_STREQ(fatigueModel.getType().c_str(), "Xia");

    // Teast limit active quantity
    {
        // Set values under 0
        fatigueModel.setState(negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest);


        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel.activeFibers(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel.fatiguedFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest);
        EXPECT_DOUBLE_EQ(fatigueModel.restingFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest+negativeFibersQuantityForFatigueXiaSetStateLimitsTest);

        // Set values over 1
        fatigueModel.setState(excessiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              0,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest-excessiveFibersQuantityForFatigueXiaSetStateLimitsTest);


        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel.activeFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest);
        EXPECT_DOUBLE_EQ(fatigueModel.fatiguedFibers(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel.restingFibers(), 0);

    }

    //Test limit resting quantity
    {
        // Set values to be corrected
        fatigueModel.setState(positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              negativeFibersQuantityForFatigueXiaSetStateLimitsTest);

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel.activeFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest+negativeFibersQuantityForFatigueXiaSetStateLimitsTest);
        EXPECT_DOUBLE_EQ(fatigueModel.fatiguedFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest);
        EXPECT_DOUBLE_EQ(fatigueModel.restingFibers(), 0);

        // Set incorrect values
        EXPECT_THROW(fatigueModel.setState(positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              excessiveFibersQuantityForFatigueXiaSetStateLimitsTest), std::runtime_error);
    }

    // Test limit fatigued quantity
    {
        // Set values
        EXPECT_THROW(fatigueModel.setState(positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest), std::runtime_error);
        EXPECT_THROW(fatigueModel.setState(positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              excessiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest), std::runtime_error);

    }
}
#endif // MODULE_MUSCLES
