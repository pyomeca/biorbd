#include <iostream>
#include <gtest/gtest.h>
#include "s2mMusculoSkeletalModel.h"

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
    s2mMusculoSkeletalModel model(modelPathForXiaDerivativeTest);
    s2mGenCoord Q(model);
    s2mGenCoord QDot(model);
    for (unsigned i = 0; i<model.nbQ(); i++){
        Q[i] = 0 ;
        QDot[i] = 0;
    }
    model.updateMuscles(model, Q, QDot, true);

    {
        std::shared_ptr<s2mMuscleHillTypeThelenFatigable> muscle(
                    std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(
                        model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<s2mMuscleFatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<s2mMuscleFatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works
        // Sanity check for the fatigue model
        EXPECT_STREQ(fatigueModel->getType().c_str(), "Xia");

        // Initial value sanity check
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), 0);

        // Apply the derivative
        s2mMuscleStateActual emg(0, activationEmgForXiaDerivativeTest); // Set target
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
        std::shared_ptr<s2mMuscleHillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<s2mMuscleFatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<s2mMuscleFatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), expectedActivationDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), expectedRestingDotForXiaDerivativeTest);
    }
}

TEST(MuscleFatigue, FatigueXiaDerivativeViaInterface){
    // Prepare the model
    s2mMusculoSkeletalModel model(modelPathForXiaDerivativeTest);
    s2mGenCoord Q(model);
    s2mGenCoord QDot(model);
    for (unsigned i = 0; i<model.nbQ(); i++){
        Q[i] = 0 ;
        QDot[i] = 0;
    }
    model.updateMuscles(model, Q, QDot, true);

    {
        std::shared_ptr<s2mMuscleHillTypeThelenFatigable> muscle(
                    std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(
                        model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works

        // Apply the derivative
        s2mMuscleStateActual emg(0, activationEmgForXiaDerivativeTest); // Set target
        muscle->fatigueState(currentActiveFibersForXiaDerivativeTest,
                             currentFatiguedFibersForXiaDerivativeTest,
                             currentRestingFibersForXiaDerivativeTest);
        muscle->computeTimeDerivativeState(emg);

    }

    // Values should be changed in the model itself
    {
        std::shared_ptr<s2mMuscleHillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(model.muscleGroup(0).muscle(0)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<s2mMuscleFatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<s2mMuscleFatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), expectedActivationDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), expectedRestingDotForXiaDerivativeTest);
    }
}

TEST(MuscleFatigue, FatigueXiaDerivativeViaCopy){
    // Prepare the model
    s2mMusculoSkeletalModel model(modelPathForXiaDerivativeTest);
    s2mGenCoord Q(model);
    s2mGenCoord QDot(model);
    for (unsigned i = 0; i<model.nbQ(); i++){
        Q[i] = 0 ;
        QDot[i] = 0;
    }
    model.updateMuscles(model, Q, QDot, true);

    {
        s2mMuscleHillTypeThelenFatigable muscle(
                    model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
        s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
        // Sanity check for the fatigue model
        EXPECT_STREQ(fatigueModel.getType().c_str(), "Xia");

        // Initial value sanity check
        EXPECT_DOUBLE_EQ(fatigueModel.activeFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel.fatiguedFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel.restingFibersDot(), 0);

        // Apply the derivative
        s2mMuscleStateActual emg(0, activationEmgForXiaDerivativeTest); // Set target
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
        std::shared_ptr<s2mMuscleHillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest)));
        EXPECT_NE(muscle, nullptr); // Expected that the cast works
        std::shared_ptr<s2mMuscleFatigueDynamicState> fatigueModel(std::dynamic_pointer_cast<s2mMuscleFatigueDynamicState>(muscle->fatigueState()));
        EXPECT_NE(fatigueModel, nullptr); // Expected that the cast works

        // Check the values
        EXPECT_DOUBLE_EQ(fatigueModel->activeFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->fatiguedFibersDot(), 0);
        EXPECT_DOUBLE_EQ(fatigueModel->restingFibersDot(), 0);
    }
}

TEST(MuscleFatigue, FatigueXiaSetStateLimitsTest){
    // Prepare the model
    s2mMusculoSkeletalModel model(modelPathForXiaDerivativeTest);
    s2mMuscleHillTypeThelenFatigable muscle(
                model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
    s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
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
