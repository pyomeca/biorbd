#include <iostream>
#include <gtest/gtest.h>

#include <rbdl/Dynamics.h>
#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#ifdef MODULE_MUSCLES
#include "Muscles/all.h"

static double requiredPrecision(1e-10);

static std::string modelPathForMuscleForce("models/arm26.bioMod");
static std::string modelPathForMuscleJacobian("models/arm26.bioMod");
static unsigned int muscleGroupForMuscleJacobian(1);
static unsigned int muscleForMuscleJacobian(1);



TEST(MuscleForce, position)
{
    // TODO
    // Position of origin, insertion, and path in local
    // Position of origin, insertion, and path in global
}

TEST(MuscleForce, force)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setOnes();
    QDot.setOnes();
    std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> states;
    for (unsigned int i=0; i<model.nbMuscleTotal(); ++i)
        states.push_back(std::make_shared<biorbd::muscles::StateDynamics>(0, 0.2));
    model.updateMuscles(Q, QDot, true);

    const biorbd::utils::Vector& F = model.musclesForces(states, false);

    Eigen::VectorXd ExpectedForce(model.nbMuscleTotal());
    ExpectedForce << 647.25276356553593, 119.55997461719004, 85.85568070134883,
            118.01635424513141, 113.18455892403414, 189.84361438713745;
    for (unsigned int i=0; i<model.nbMuscleTotal(); ++i)
        EXPECT_NEAR(F(i), ExpectedForce(i), requiredPrecision);
}

TEST(MuscleForce, torqueFromMuscles)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    biorbd::rigidbody::GeneralizedAcceleration QDDot(model), QDDotExpected(model);
    Q.setOnes()/10;
    QDot.setOnes()/10;
    std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> states;
    for (unsigned int i=0; i<model.nbMuscleTotal(); ++i)
        states.push_back(std::make_shared<biorbd::muscles::StateDynamics>(0, 0.2));

    biorbd::rigidbody::GeneralizedTorque Tau(model), TauExpected(model);
    TauExpected << -18.271389285751727, -7.820566757538376;
    Tau = model.muscularJointTorque(states, true, &Q, &QDot);
    for (unsigned int i=0; i<QDDot.size(); ++i)
        EXPECT_NEAR(Tau[i], TauExpected[i], requiredPrecision);

    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);
    QDDotExpected << -2.4941551687243537, 0.04600953825654895;
    for (unsigned int i=0; i<QDDot.size(); ++i)
        EXPECT_NEAR(QDDot[i], QDDotExpected[i], requiredPrecision);

}

TEST(MuscleJacobian, jacobian){
    biorbd::Model model(modelPathForMuscleJacobian);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes()/10;

    // Force computation of geometry
    biorbd::muscles::Muscle& muscle(model.muscleGroup(muscleForMuscleJacobian).muscle(muscleGroupForMuscleJacobian));
    EXPECT_THROW(muscle.position().jacobian(), std::runtime_error);
    model.updateMuscles(Q, true);

    unsigned int nRows(3 * (muscle.pathModifier().nbObjects() + 2));
    biorbd::utils::Matrix jacoRef(nRows, model.nbQ());
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.13689690274955996;    jacoRef(0, 1) = 0;
    jacoRef(1, 0) = 0.0048155626519999824; jacoRef(1, 1) = 0;
    jacoRef(2, 0) = 0.0080659044260533528;  jacoRef(2, 1) = 0;
    jacoRef(3, 0) = 0.1530001664829288;    jacoRef(3, 1) = 0;
    jacoRef(4, 0) = -0.011356405676791911;  jacoRef(4, 1) = 0;
    jacoRef(5, 0) = 0.0090532669706573556;  jacoRef(5, 1) = 0;
    jacoRef(6, 0) = 0.22806179660848588;    jacoRef(6, 1) = 0;
    jacoRef(7, 0) = -0.0097422466736260017;  jacoRef(7, 1) = 0;
    jacoRef(8, 0) = 0.013478229213671932;    jacoRef(8, 1) = 0;
    jacoRef(9, 0) = 0.2675513197235147;    jacoRef(9, 1) = 0;
    jacoRef(10, 0) = 0.0086890497375387721; jacoRef(10, 1) = 0;
    jacoRef(11, 0) = 0.015765668988435677;  jacoRef(11, 1) = 0;
    jacoRef(12, 0) = 0.28198829726211816;   jacoRef(12, 1) = -0.0059281232164749703;
    jacoRef(13, 0) = 0.010784640478103213; jacoRef(13, 1) = -0.023460590547529164;
    jacoRef(14, 0) = 0.016612631083717935;  jacoRef(14, 1) = 0.0013801799073302687;

    // Compare with computed values
    biorbd::utils::Matrix jaco(muscle.position().jacobian());
    for (unsigned int i=0; i<jaco.rows(); ++i)
        for (unsigned int j=0; j<jaco.cols(); ++j)
            EXPECT_NEAR(jaco(i, j), jacoRef(i, j), requiredPrecision);

    // Change Q
    Q.setOnes();
    model.updateMuscles(Q, true);
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.08134540996216065;    jacoRef(0, 1) = 0;
    jacoRef(1, 0) = 0.11041411771499073; jacoRef(1, 1) = 0;
    jacoRef(2, 0) = 0.0045450332887922848;  jacoRef(2, 1) = 0;
    jacoRef(3, 0) = 0.10400305103312649;    jacoRef(3, 1) = 0;
    jacoRef(4, 0) = 0.1129992026935549;  jacoRef(4, 1) = 0;
    jacoRef(5, 0) = 0.0058758916739148634;  jacoRef(5, 1) = 0;
    jacoRef(6, 0) = 0.14940786497893349;    jacoRef(6, 1) = 0;
    jacoRef(7, 0) = 0.17290229814238411;  jacoRef(7, 1) = 0;
    jacoRef(8, 0) = 0.008416785894586807;    jacoRef(8, 1) = 0;
    jacoRef(9, 0) = 0.1595464389865561;    jacoRef(9, 1) = 0;
    jacoRef(10, 0) = 0.21534433421098692; jacoRef(10, 1) = 0;
    jacoRef(11, 0) = 0.0089171811535144387;  jacoRef(11, 1) = 0;
    jacoRef(12, 0) = 0.17634886417858722;   jacoRef(12, 1) = 0.024220863439657705;
    jacoRef(13, 0) = 0.24680272575831863; jacoRef(13, 1) = -0.00046752133823085851;
    jacoRef(14, 0) = 0.0098360540170798361;  jacoRef(14, 1) = 0.00076029489073317982;

    // Compare with computed values
    jaco = muscle.position().jacobian();
    for (unsigned int i=0; i<jaco.rows(); ++i)
        for (unsigned int j=0; j<jaco.cols(); ++j)
            EXPECT_NEAR(jaco(i, j), jacoRef(i, j), requiredPrecision);
}
TEST(MuscleJacobian, jacobianLength){
    biorbd::Model model(modelPathForMuscleJacobian);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes()/10;
    model.updateMuscles(Q, true);

    unsigned int nRows(model.nbMuscleTotal());
    biorbd::utils::Matrix jacoRef(nRows, model.nbQ());
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.037620360527045288;      jacoRef(0, 1) = 0.022230374109936522;
    jacoRef(1, 0) = -0.017006708057341259;     jacoRef(1, 1) = -0.012779004692822197;
    jacoRef(2, 0) = -0.0016529882883160136;    jacoRef(2, 1) = -0.012779004692822197;
    jacoRef(3, 0) =  2.6526502766149064e-17;   jacoRef(3, 1) = 0.022230374109936522;
    jacoRef(4, 0) =  5.2979083494670027e-18;   jacoRef(4, 1) = 0.022230374109936522;
    jacoRef(5, 0) =  5.3888967663972615e-18;   jacoRef(5, 1) = 0.00064415763125857495;

    // Compare with computed values
    biorbd::utils::Matrix jaco(model.musclesLengthJacobian(Q));
    for (unsigned int i=0; i<jaco.rows(); ++i)
        for (unsigned int j=0; j<jaco.cols(); ++j)
            EXPECT_NEAR(jaco(i, j), jacoRef(i, j), requiredPrecision);

    // Change Q
    Q.setOnes();
    model.updateMuscles(Q, true);
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.025994784772561841;      jacoRef(0, 1) = 0.021586396946686921;
    jacoRef(1, 0) = -0.011088003983559729;     jacoRef(1, 1) = -0.039462418895871744;
    jacoRef(2, 0) = 0.032285277521606391;    jacoRef(2, 1) = -0.039462418895871744;
    jacoRef(3, 0) =  1.1900679209044559e-19;   jacoRef(3, 1) = 0.021586396946686921;
    jacoRef(4, 0) =  1.4848744796707674e-17;   jacoRef(4, 1) = 0.021586396946686921;
    jacoRef(5, 0) =  -2.5088055592238924e-17;   jacoRef(5, 1) = -0.015991501107813871;
    jaco = model.musclesLengthJacobian(Q);
        for (unsigned int i=0; i<jaco.rows(); ++i)
            for (unsigned int j=0; j<jaco.cols(); ++j)
                EXPECT_NEAR(jaco(i, j), jacoRef(i, j), requiredPrecision);
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
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicState& fatigueModel(dynamic_cast<biorbd::muscles::FatigueDynamicState&>(muscle.fatigueState()));
        // Sanity check for the fatigue model
        EXPECT_EQ(fatigueModel.getType(), biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

        // Initial value sanity check
        EXPECT_NEAR(fatigueModel.activeFibersDot(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(), 0, requiredPrecision);

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0, activationEmgForXiaDerivativeTest); // Set target
        fatigueModel.setState(currentActiveFibersForXiaDerivativeTest,
                               currentFatiguedFibersForXiaDerivativeTest,
                               currentRestingFibersForXiaDerivativeTest);
        fatigueModel.timeDerivativeState(emg, model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest).characteristics());

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(), expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(), expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }

    // Values should be changed in the model itself
    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicState& fatigueModel(dynamic_cast<biorbd::muscles::FatigueDynamicState&>(muscle.fatigueState()));

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(), expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(), expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }
}

TEST(MuscleFatigue, FatigueXiaDerivativeViaInterface){
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0, activationEmgForXiaDerivativeTest); // Set target
        muscle.setFatigueState(currentActiveFibersForXiaDerivativeTest,
                             currentFatiguedFibersForXiaDerivativeTest,
                             currentRestingFibersForXiaDerivativeTest);
        muscle.computeTimeDerivativeState(emg);

    }

    // Values should be changed in the model itself
    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(0).muscle(0));
        biorbd::muscles::FatigueDynamicState& fatigueModel(dynamic_cast<biorbd::muscles::FatigueDynamicState&>(muscle.fatigueState()));

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(), expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(), expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }
}

TEST(MuscleFatigue, FatigueXiaDerivativeShallowViaCopy){
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillThelenTypeFatigable muscle(
                    model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicStateXia fatigueModel(dynamic_cast<biorbd::muscles::FatigueDynamicStateXia&>(muscle.fatigueState()));
        // Sanity check for the fatigue model
        EXPECT_EQ(fatigueModel.getType(), biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

        // Initial value sanity check
        EXPECT_NEAR(fatigueModel.activeFibersDot(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(), 0, requiredPrecision);

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0, activationEmgForXiaDerivativeTest); // Set target
        fatigueModel.setState(currentActiveFibersForXiaDerivativeTest,
                              currentFatiguedFibersForXiaDerivativeTest,
                              currentRestingFibersForXiaDerivativeTest);
        fatigueModel.timeDerivativeState(emg, model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest).characteristics());

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(), expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(), expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }

    // Values should be changed in the model itself since everything is shallowcopied
    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicState& fatigueModel(dynamic_cast<biorbd::muscles::FatigueDynamicState&>(muscle.fatigueState()));

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(), expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(), expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(), expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }
}

TEST(MuscleFatigue, FatigueXiaSetStateLimitsTest){
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::muscles::HillThelenTypeFatigable muscle(
                model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(muscleForXiaDerivativeTest));
    biorbd::muscles::FatigueDynamicStateXia fatigueModel(dynamic_cast<biorbd::muscles::FatigueDynamicStateXia&>(muscle.fatigueState()));
    // Sanity check for the fatigue model
    EXPECT_EQ(fatigueModel.getType(), biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    // Teast limit active quantity
    {
        // Set values under 0
        fatigueModel.setState(negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest);


        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibers(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest+negativeFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);

        // Set values over 1
        fatigueModel.setState(excessiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              0,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest-excessiveFibersQuantityForFatigueXiaSetStateLimitsTest);


        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibers(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibers(), 0, requiredPrecision);

    }

    //Test limit resting quantity
    {
        // Set values to be corrected
        fatigueModel.setState(positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              negativeFibersQuantityForFatigueXiaSetStateLimitsTest);

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest+negativeFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibers(), positiveFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibers(), 0, requiredPrecision);

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
