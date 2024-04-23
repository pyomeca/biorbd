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
#include "RigidBody/NodeSegment.h"
#include "InternalForces/Ligaments/all.h"
#include "InternalForces/all.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

using namespace BIORBD_NAMESPACE;

static double requiredPrecision(1e-10);

static std::string modelPathForGenericTest("models/arm26_WithLigaments.bioMod");

TEST(Ligaments, size)
{
    Model model(modelPathForGenericTest);
    size_t nbLig(model.nbLigaments());
    EXPECT_EQ(nbLig, 3);
    EXPECT_EQ(model.ligaments().size(), nbLig);
}

static size_t ligamentConstantType(0);
TEST(constant, unitTest)
{
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant ligamentConstant(
            model.ligament(ligamentConstantType));
        ligamentConstant.setName("newName");
        EXPECT_STREQ(ligamentConstant.name().c_str(), "newName");
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant ligamentConstant(
            model.ligament(ligamentConstantType));
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;
        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

        ligamentConstant.updateOrientations(updatedModel, Q, Qdot);
        SCALAR_TO_DOUBLE(fl, ligamentConstant.Fl());
        SCALAR_TO_DOUBLE(damping, ligamentConstant.damping());
        EXPECT_NEAR(fl, 500, requiredPrecision);
        EXPECT_NEAR(damping, 0.00056194583866446676, requiredPrecision);

        // with damping
        internal_forces::ligaments::LigamentCharacteristics charac(ligamentConstant.characteristics());
        SCALAR_TO_DOUBLE(forceDamped, ligamentConstant.force());
        EXPECT_NEAR(forceDamped, 500.00056194583868, requiredPrecision);

        // without damping
        charac.setDampingParam(0);
        SCALAR_TO_DOUBLE(force, ligamentConstant.force());
        EXPECT_NEAR(force, 500, requiredPrecision);
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant originalLigament(
            model.ligament(ligamentConstantType));

        internal_forces::ligaments::LigamentConstant newLigament(
            50,
            "newName",
            originalLigament.position(),
            originalLigament.characteristics());

        EXPECT_STREQ(newLigament.name().c_str(), "newName");
        EXPECT_EQ(newLigament.type(), internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_CONSTANT);
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant originalLigament(
            model.ligament(ligamentConstantType));

        internal_forces::ligaments::LigamentConstant newLigament(
            50,
            "newName",
            originalLigament.position(),
            originalLigament.characteristics(),
            originalLigament.pathModifier());

        EXPECT_STREQ(newLigament.name().c_str(), "newName");
        EXPECT_EQ(newLigament.type(), internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_CONSTANT);

    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant originalLigament(
             model.ligament(ligamentConstantType));

        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;
        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

        SCALAR_TO_DOUBLE(force, originalLigament.force(updatedModel, Q, Qdot, true));
        EXPECT_NEAR(force, 500.00056194583868, requiredPrecision);
    }
}

TEST(constant, copy)
{
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant ligamentConstant(
             model.ligament(ligamentConstantType));
        rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;

        internal_forces::ligaments::LigamentConstant shallowCopy(ligamentConstant);
        internal_forces::ligaments::LigamentConstant deepCopyNow(ligamentConstant.DeepCopy());
        internal_forces::ligaments::LigamentConstant deepCopyLater;
        deepCopyLater.DeepCopy(ligamentConstant);

        utils::String originalName(ligamentConstant.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

        utils::String newName("MyNewLigamentName");
        ligamentConstant.setName(newName);
        EXPECT_STREQ(ligamentConstant.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant ligamentConstant(
             model.ligament(ligamentConstantType));
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;

        auto updatedModel = model.UpdateKinematicsCustom(&Q);
        ligamentConstant.updateOrientations(updatedModel, Q, Qdot);
        
        internal_forces::ligaments::LigamentConstant shallowCopy(ligamentConstant);
        internal_forces::ligaments::LigamentConstant deepCopyNow(ligamentConstant.DeepCopy());
        internal_forces::ligaments::LigamentConstant deepCopyLater;
        deepCopyLater.DeepCopy(ligamentConstant);

        {
            SCALAR_TO_DOUBLE(length, ligamentConstant.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.14927574026595969, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.14927574026595969, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.14927574026595969, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.14927574026595969, requiredPrecision);
        }

        // Change the position of the insertion and pennation angle and compare again (length and insertion in Local)
        utils::Vector3d insertion(ligamentConstant.position().insertionInLocal());
        insertion.set(0.5, 0.6, 0.7);
        const_cast<internal_forces::Geometry&>
        (ligamentConstant.position()).setInsertionInLocal(insertion);
        utils::String oldName(insertion.utils::Node::name());
        utils::String newName("MyNewName");
        insertion.setName(newName);

        ligamentConstant.updateOrientations(updatedModel, Q, Qdot);        
        {
            SCALAR_TO_DOUBLE(length, ligamentConstant.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 1.1421654476456886, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 1.1421654476456886, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.14927574026595969, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.14927574026595969, requiredPrecision);
            EXPECT_EQ(ligamentConstant.position().insertionInLocal().utils::Node::name(),
                      newName);
            EXPECT_EQ(shallowCopy.position().insertionInLocal().utils::Node::name(),
                      newName);
            EXPECT_EQ(deepCopyNow.position().insertionInLocal().utils::Node::name(),
                      oldName);
            EXPECT_EQ(
                deepCopyLater.position().insertionInLocal().utils::Node::name(),
                oldName);
        }
    }
}

static size_t ligamentSpringLinearType(1);
TEST(springLinear, unitTest)
{
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringLinear ligamentSpringLinear(
            model.ligament(ligamentSpringLinearType));
        ligamentSpringLinear.setName("newName");
        EXPECT_STREQ(ligamentSpringLinear.name().c_str(), "newName");
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringLinear ligamentSpringLinear(
            model.ligament(ligamentSpringLinearType));
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;
        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);
        ligamentSpringLinear.updateOrientations(updatedModel, Q, Qdot);

        SCALAR_TO_DOUBLE(fl, ligamentSpringLinear.Fl());
        SCALAR_TO_DOUBLE(damping, ligamentSpringLinear.damping());
        EXPECT_NEAR(fl, 27.51715156544391, requiredPrecision);
        EXPECT_NEAR(damping, 3.2207881562929018e-05, requiredPrecision);

        // with damping
        internal_forces::ligaments::LigamentCharacteristics charac(ligamentSpringLinear.characteristics());
        SCALAR_TO_DOUBLE(forceDamped, ligamentSpringLinear.force());
        EXPECT_NEAR(forceDamped, 27.517183773325474, requiredPrecision);

        // without damping
        charac.setDampingParam(0);
        SCALAR_TO_DOUBLE(force, ligamentSpringLinear.force());
        EXPECT_NEAR(force, 27.51715156544391, requiredPrecision);
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringLinear ligamentSpringLinear(
            model.ligament(ligamentSpringLinearType));

        internal_forces::ligaments::LigamentSpringLinear newLigament(
            50,
            "newName",
            ligamentSpringLinear.position(),
            ligamentSpringLinear.characteristics());

        EXPECT_STREQ(newLigament.name().c_str(), "newName");
        EXPECT_EQ(newLigament.type(), internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_LINEAR);
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringLinear originalLigament(
            model.ligament(ligamentSpringLinearType));

        internal_forces::ligaments::LigamentSpringLinear newLigament(
            50,
            "newName",
            originalLigament.position(),
            originalLigament.characteristics(),
            originalLigament.pathModifier());

        EXPECT_STREQ(newLigament.name().c_str(), "newName");
        EXPECT_EQ(newLigament.type(), internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_LINEAR);

    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringLinear originalLigament(
             model.ligament(ligamentSpringLinearType));

        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;
        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

        SCALAR_TO_DOUBLE(force, originalLigament.force(updatedModel, Q, Qdot, true));
        EXPECT_NEAR(force, 27.517183773325474, requiredPrecision);
    }
}

TEST(springLinear, copy)
{
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringLinear ligamentSpringLinear(
             model.ligament(ligamentSpringLinearType));
        rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;
        
        internal_forces::ligaments::LigamentSpringLinear shallowCopy(ligamentSpringLinear);
        internal_forces::ligaments::LigamentSpringLinear deepCopyNow(ligamentSpringLinear.DeepCopy());
        internal_forces::ligaments::LigamentSpringLinear deepCopyLater;
        deepCopyLater.DeepCopy(ligamentSpringLinear);

        utils::String originalName(ligamentSpringLinear.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
        
        {
            SCALAR_TO_DOUBLE(ligamentSpringLinearStiffness, ligamentSpringLinear.stiffness());
            SCALAR_TO_DOUBLE(shallowCopyStiffness, shallowCopy.stiffness());
            SCALAR_TO_DOUBLE(deepCopyNowStiffness, deepCopyNow.stiffness());
            SCALAR_TO_DOUBLE(deepCopyLaterStiffness, deepCopyLater.stiffness());
            EXPECT_FLOAT_EQ(ligamentSpringLinearStiffness, 500);
            EXPECT_FLOAT_EQ(shallowCopyStiffness, 500);
            EXPECT_FLOAT_EQ(deepCopyNowStiffness, 500);
            EXPECT_FLOAT_EQ(deepCopyLaterStiffness, 500);
        }

        utils::String newName("MyNewMuscleName");
        ligamentSpringLinear.setName(newName);
        EXPECT_STREQ(ligamentSpringLinear.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
        
        ligamentSpringLinear.setStiffness(100);
        {
            SCALAR_TO_DOUBLE(ligamentSpringLinearStiffness, ligamentSpringLinear.stiffness());
            SCALAR_TO_DOUBLE(shallowCopyStiffness, shallowCopy.stiffness());
            SCALAR_TO_DOUBLE(deepCopyNowStiffness, deepCopyNow.stiffness());
            SCALAR_TO_DOUBLE(deepCopyLaterStiffness, deepCopyLater.stiffness());
            EXPECT_FLOAT_EQ(ligamentSpringLinearStiffness, 100);
            EXPECT_FLOAT_EQ(shallowCopyStiffness, 100);
            EXPECT_FLOAT_EQ(deepCopyNowStiffness, 500);
            EXPECT_FLOAT_EQ(deepCopyLaterStiffness, 500);
        }
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringLinear ligamentSpringLinear(
             model.ligament(ligamentSpringLinearType));
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;

        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);
        ligamentSpringLinear.updateOrientations(updatedModel, Q);

        internal_forces::ligaments::LigamentSpringLinear shallowCopy(ligamentSpringLinear);
        internal_forces::ligaments::LigamentSpringLinear deepCopyNow(ligamentSpringLinear.DeepCopy());
        internal_forces::ligaments::LigamentSpringLinear deepCopyLater;
        deepCopyLater.DeepCopy(ligamentSpringLinear);

        {
            SCALAR_TO_DOUBLE(length, ligamentSpringLinear.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.14083430313088782, requiredPrecision);
        }

        // Change the position of the insertion and pennation angle and compare again (length and insertion in Local)
        utils::Vector3d insertion(ligamentSpringLinear.position().insertionInLocal());
        insertion.set(0.5, 0.6, 0.7);
        const_cast<internal_forces::Geometry&>(ligamentSpringLinear.position()).setInsertionInLocal(insertion);
        utils::String oldName(insertion.utils::Node::name());
        utils::String newName("MyNewName");
        insertion.setName(newName);
        ligamentSpringLinear.updateOrientations(updatedModel, Q, Qdot);

        {
            SCALAR_TO_DOUBLE(length, ligamentSpringLinear.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.97513625610637888, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.97513625610637888, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.14083430313088782, requiredPrecision);
            EXPECT_EQ(ligamentSpringLinear.position().insertionInLocal().utils::Node::name(), newName);
            EXPECT_EQ(shallowCopy.position().insertionInLocal().utils::Node::name(), newName);
            EXPECT_EQ(deepCopyNow.position().insertionInLocal().utils::Node::name(), oldName);
            EXPECT_EQ(deepCopyLater.position().insertionInLocal().utils::Node::name(), oldName);
        }
    }
}

static size_t ligamentSpringSecondOrderType(2);
TEST(springSecondOrder, unitTest)
{
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringSecondOrder ligamentSpringSecondOrder(
            model.ligament(ligamentSpringSecondOrderType));
        ligamentSpringSecondOrder.setName("newName");
        EXPECT_STREQ(ligamentSpringSecondOrder.name().c_str(), "newName");
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringSecondOrder ligamentSpringSecondOrder(
            model.ligament(ligamentSpringSecondOrderType));
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;

        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);
        ligamentSpringSecondOrder.updateOrientations(updatedModel, Q, Qdot);

        SCALAR_TO_DOUBLE(fl, ligamentSpringSecondOrder.Fl());
        SCALAR_TO_DOUBLE(damping, ligamentSpringSecondOrder.damping());
        EXPECT_NEAR(fl, 139.51348983210974, requiredPrecision);
        EXPECT_NEAR(damping, 3.8649457875514822e-05, requiredPrecision);

        // with damping
        internal_forces::ligaments::LigamentCharacteristics charac(ligamentSpringSecondOrder.characteristics());
        SCALAR_TO_DOUBLE(forceDamped, ligamentSpringSecondOrder.force());
        EXPECT_NEAR(forceDamped, 139.51352848156762, requiredPrecision);

        // without damping
        charac.setDampingParam(0);
        SCALAR_TO_DOUBLE(force, ligamentSpringSecondOrder.force());
        EXPECT_NEAR(force, 139.51348983210974, requiredPrecision);
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringSecondOrder ligamentSpringSecondOrder(
            model.ligament(ligamentSpringSecondOrderType));

        internal_forces::ligaments::LigamentSpringSecondOrder newLigament(
            50,
            0.5,
            "newName",
            ligamentSpringSecondOrder.position(),
            ligamentSpringSecondOrder.characteristics());

        EXPECT_STREQ(newLigament.name().c_str(), "newName");
        EXPECT_EQ(newLigament.type(), internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_SECOND_ORDER);
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringSecondOrder originalLigament(
            model.ligament(ligamentSpringSecondOrderType));

        internal_forces::ligaments::LigamentSpringSecondOrder newLigament(
            50,
            0.5,
            "newName",
            originalLigament.position(),
            originalLigament.characteristics(),
            originalLigament.pathModifier());

        EXPECT_STREQ(newLigament.name().c_str(), "newName");
        EXPECT_EQ(newLigament.type(), internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_SECOND_ORDER);

    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringSecondOrder originalLigament(
             model.ligament(ligamentSpringSecondOrderType));

        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;
        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);
        SCALAR_TO_DOUBLE(force, originalLigament.force(updatedModel, Q, Qdot, true));
        EXPECT_NEAR(force, 139.51352848156762, requiredPrecision);
    }
}

TEST(springSecondOrder, copy)
{
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringSecondOrder ligamentSpringSecondOrder(
             model.ligament(ligamentSpringSecondOrderType));
        rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;

        internal_forces::ligaments::LigamentSpringSecondOrder shallowCopy(ligamentSpringSecondOrder);
        internal_forces::ligaments::LigamentSpringSecondOrder deepCopyNow(ligamentSpringSecondOrder.DeepCopy());
        internal_forces::ligaments::LigamentSpringSecondOrder deepCopyLater;
        deepCopyLater.DeepCopy(ligamentSpringSecondOrder);

        utils::String originalName(ligamentSpringSecondOrder.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

        utils::String newName("MyNewMuscleName");
        ligamentSpringSecondOrder.setName(newName);
        EXPECT_STREQ(ligamentSpringSecondOrder.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
    }
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentSpringSecondOrder ligamentSpringSecondOrder(
             model.ligament(ligamentSpringSecondOrderType));
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        Q = Q.setOnes() / 10;
        Qdot = Qdot.setOnes() / 10;

        auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);
        ligamentSpringSecondOrder.updateOrientations(updatedModel, Q);

        internal_forces::ligaments::LigamentSpringSecondOrder shallowCopy(ligamentSpringSecondOrder);
        internal_forces::ligaments::LigamentSpringSecondOrder deepCopyNow(ligamentSpringSecondOrder.DeepCopy());
        internal_forces::ligaments::LigamentSpringSecondOrder deepCopyLater;
        deepCopyLater.DeepCopy(ligamentSpringSecondOrder);

        {
            SCALAR_TO_DOUBLE(length, ligamentSpringSecondOrder.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.14083430313088782, requiredPrecision);
        }

        // Change the position of the insertion and pennation angle and compare again (length and insertion in Local)
        utils::Vector3d insertion(ligamentSpringSecondOrder.position().insertionInLocal());
        insertion.set(0.5, 0.6, 0.7);
        const_cast<internal_forces::Geometry&>
        (ligamentSpringSecondOrder.position()).setInsertionInLocal(insertion);
        utils::String oldName(insertion.utils::Node::name());
        utils::String newName("MyNewName");
        insertion.setName(newName);
        ligamentSpringSecondOrder.updateOrientations(updatedModel, Q, Qdot);


        {
            SCALAR_TO_DOUBLE(length, ligamentSpringSecondOrder.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.97513625610637888, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.97513625610637888, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.14083430313088782, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.14083430313088782, requiredPrecision);
            EXPECT_EQ(ligamentSpringSecondOrder.position().insertionInLocal().utils::Node::name(),
                      newName);
            EXPECT_EQ(shallowCopy.position().insertionInLocal().utils::Node::name(),
                      newName);
            EXPECT_EQ(deepCopyNow.position().insertionInLocal().utils::Node::name(),
                      oldName);
            EXPECT_EQ(
                deepCopyLater.position().insertionInLocal().utils::Node::name(),
                oldName);
        }
    }
}

TEST(Ligaments, AddLigaments)
{
    {
        Model model(modelPathForGenericTest);
        internal_forces::ligaments::LigamentConstant LigamentToAdd;
        model.addLigament(LigamentToAdd);

        //Check number of ligament
        EXPECT_EQ(model.nbLigaments(), 4);
    }
    {
        Model model(modelPathForGenericTest);
        //Check number of ligament
        EXPECT_EQ(model.nbLigaments(), 3);

    }
}

TEST(LigamentForce, force)
{
    Model model(modelPathForGenericTest);
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes()/10;
    Qdot = Qdot.setOnes()/10;
    
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);
    const utils::Vector& F = model.ligamentForces(updatedModel, Q, Qdot);

    std::vector<double> ExpectedForce({500.00056194583868, 27.517183773325474, 139.51352848156762});
    for (unsigned int i=0; i<model.nbLigaments(); ++i) {
        SCALAR_TO_DOUBLE(val, F(i));
        EXPECT_NEAR(val, ExpectedForce[i], requiredPrecision);
    }
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(LigamentForce, forceUpdateOutside)
{
    Model model(modelPathForGenericTest);
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes()/10;
    Qdot = Qdot.setOnes()/10;
    model.updateLigaments(Q, Qdot, true);

    const utils::Vector& F = model.ligamentForces(Q, Qdot, false);

    std::vector<double> ExpectedForce({500.00056194583868, 27.517183773325474, 139.51352848156762});
    for (unsigned int i=0; i<model.nbLigaments(); ++i) {
        SCALAR_TO_DOUBLE(val, F(i));
        EXPECT_NEAR(val, ExpectedForce[i], requiredPrecision);
    }
}
#endif

TEST(LigamentTorque, torqueFromLigaments)
{
    Model model(modelPathForGenericTest);
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    rigidbody::GeneralizedAcceleration Qddot(model);
    Q.setOnes()/10;
    Qdot.setOnes()/10;
    rigidbody::GeneralizedTorque Tau(model);
    std::vector<double> TauExpected({8.4576580134417226e-15, 3.0375576471800541});
    Tau = model.ligamentsJointTorque(Q, Qdot);
    for (unsigned int i=0; i<Qddot.size(); ++i) {
        SCALAR_TO_DOUBLE(val, Tau(i));
        EXPECT_NEAR(val, TauExpected[i], requiredPrecision);
    }

    RigidBodyDynamics::ForwardDynamics(model, Q, Qdot, Tau, Qddot);
    std::vector<double> QddotExpected({-29.605664255664376, 94.507026107190669});
    for (unsigned int i=0; i<Qddot.size(); ++i) {
        SCALAR_TO_DOUBLE(val, Qddot(i));
        EXPECT_NEAR(val, QddotExpected[i], requiredPrecision);
    }
}

TEST(LigamentCharacterics, unittest)
{
    {
        internal_forces::ligaments::LigamentCharacteristics charact;
        {
            SCALAR_TO_DOUBLE(slackLength, charact.ligamentSlackLength());
            EXPECT_NEAR(slackLength, 0, requiredPrecision);
        }
        double newSlackLength(3.4);
        charact.setLigamentSlackLength(newSlackLength);
        {
            SCALAR_TO_DOUBLE(slackLength, charact.ligamentSlackLength());
            EXPECT_NEAR(slackLength, newSlackLength, requiredPrecision);
        }
    }

    {
        internal_forces::ligaments::LigamentCharacteristics charact;
        {
            SCALAR_TO_DOUBLE(maxSpeed, charact.maxShorteningSpeed());
            EXPECT_NEAR(maxSpeed, 1, requiredPrecision);
        }
        double newMaxSpeed(156.9);
        charact.setMaxShorteningSpeed(newMaxSpeed);

        {
            SCALAR_TO_DOUBLE(maxSpeed, charact.maxShorteningSpeed());
            EXPECT_NEAR(maxSpeed, newMaxSpeed, requiredPrecision);
        }
    }

    {
        internal_forces::ligaments::LigamentCharacteristics charact;
        {
            SCALAR_TO_DOUBLE(dampingFactor, charact.dampingParam());
            EXPECT_NEAR(dampingFactor, 0, requiredPrecision);
        }
        double newDampingFactor(5.3);
        charact.setDampingParam(newDampingFactor);
        {
            SCALAR_TO_DOUBLE(dampingFactor, charact.dampingParam());
            EXPECT_NEAR(dampingFactor, newDampingFactor, requiredPrecision);
        }

    }
}


