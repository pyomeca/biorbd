#include "biorbdConfig.h"

#include <gtest/gtest.h>
#include <iostream>

#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "InternalForces/Muscles/all.h"
#include "InternalForces/all.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/NodeSegment.h"
#include "Utils/Matrix.h"
#include "Utils/RotoTrans.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

static double requiredPrecision(1e-10);

static std::string modelPathForMuscleForce("models/arm26.bioMod");
static std::string modelPathForBuchananDynamics("models/arm26_buchanan.bioMod");
static std::string modelPathForDeGrooteDynamics("models/arm26_degroote.bioMod");
static std::string modelPathForMuscleJacobian("models/arm26.bioMod");
static size_t muscleGroupForMuscleJacobian(1);
static size_t muscleForMuscleJacobian(1);

static size_t muscleGroupForIdealizedActuator(1);
static size_t muscleForIdealizedActuator(1);

TEST(Muscles, size) {
  Model model(modelPathForMuscleForce);
  size_t nbMus(model.nbMuscles());
  EXPECT_EQ(nbMus, 6);
  EXPECT_EQ(nbMus, model.nbMuscleTotal());
  EXPECT_EQ(model.muscles().size(), nbMus);
  int cmp(0);
  for (auto g : model.muscleGroups()) {
    cmp += static_cast<int>(g.nbMuscles());
  }
  EXPECT_EQ(nbMus, cmp);

  cmp = 0;
  for (auto g : model.muscleGroups()) {
    for (auto m : g.muscles()) {
      EXPECT_STREQ(m->name().c_str(), model.muscle(cmp).name().c_str());
      EXPECT_STREQ(m->name().c_str(), model.muscles()[cmp]->name().c_str());
      cmp++;
    }
  }

  EXPECT_THROW(model.muscle(cmp).name(), std::runtime_error);
}

TEST(IdealizedActuator, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));
    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q);

    SCALAR_TO_DOUBLE(length, idealizedActuator.length(updatedModel, Q, true));
    SCALAR_TO_DOUBLE(
        musculoTendonLength,
        idealizedActuator.musculoTendonLength(updatedModel, Q, true));
    EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
    EXPECT_NEAR(musculoTendonLength, 0.1563647052655904, requiredPrecision);
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));
    idealizedActuator.setName("nom");
    EXPECT_STREQ(idealizedActuator.name().c_str(), "nom");
  }

  {
    Model model(modelPathForMuscleForce);
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));
    SCALAR_TO_DOUBLE(
        velocityTrue, idealizedActuator.velocity(updatedModel, Q, Qdot, true));
    SCALAR_TO_DOUBLE(
        velocityFalse,
        idealizedActuator.velocity(updatedModel, Q, Qdot, false));
    EXPECT_NEAR(velocityTrue, 0.0022230374109936529, requiredPrecision);
    EXPECT_NEAR(velocityFalse, 0.0022230374109936529, requiredPrecision);
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    static double activationEmgForHillTypeTest(1);
    internal_forces::muscles::StateDynamics emg(
        0, activationEmgForHillTypeTest);

    SCALAR_TO_DOUBLE(emg1, idealizedActuator.force(emg));
    SCALAR_TO_DOUBLE(emg2, idealizedActuator.force(model, Q, emg));
    SCALAR_TO_DOUBLE(emg3, idealizedActuator.force(model, Q, Qdot, emg));
    EXPECT_NEAR(emg1, 624.29999999999995, requiredPrecision);
    EXPECT_NEAR(emg2, 624.29999999999995, requiredPrecision);
    EXPECT_NEAR(emg3, 624.29999999999995, requiredPrecision);
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuatorOrigin(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));

    internal_forces::muscles::IdealizedActuator idealizedActuatorNew(
        "newName",
        idealizedActuatorOrigin.position(),
        idealizedActuatorOrigin.characteristics());

    EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
    EXPECT_EQ(
        idealizedActuatorNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuatorOrigin(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));

    internal_forces::muscles::IdealizedActuator idealizedActuatorNew(
        "newName",
        idealizedActuatorOrigin.position(),
        idealizedActuatorOrigin.characteristics(),
        idealizedActuatorOrigin.state());

    EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
    EXPECT_EQ(
        idealizedActuatorNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuatorOrigin(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));

    internal_forces::muscles::IdealizedActuator idealizedActuatorNew(
        "newName",
        idealizedActuatorOrigin.position(),
        idealizedActuatorOrigin.characteristics());

    EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
    EXPECT_EQ(
        idealizedActuatorNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuatorOrigin(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));

    internal_forces::muscles::IdealizedActuator idealizedActuatorNew(
        "newName",
        idealizedActuatorOrigin.position(),
        idealizedActuatorOrigin.characteristics(),
        idealizedActuatorOrigin.pathModifier());

    EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
    EXPECT_EQ(
        idealizedActuatorNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuatorOrigin(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));

    internal_forces::muscles::IdealizedActuator idealizedActuatorNew(
        "newName",
        idealizedActuatorOrigin.position(),
        idealizedActuatorOrigin.characteristics(),
        idealizedActuatorOrigin.pathModifier(),
        idealizedActuatorOrigin.state());

    EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
    EXPECT_EQ(
        idealizedActuatorNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
  }
}

TEST(IdealizedActuator, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));
    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::IdealizedActuator shallowCopy(idealizedActuator);
    internal_forces::muscles::IdealizedActuator deepCopyNow(
        idealizedActuator.DeepCopy());
    internal_forces::muscles::IdealizedActuator deepCopyLater;
    deepCopyLater.DeepCopy(idealizedActuator);

    utils::String originalName(idealizedActuator.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    idealizedActuator.setName(newName);
    EXPECT_STREQ(idealizedActuator.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));

    internal_forces::muscles::IdealizedActuator shallowCopy(idealizedActuator);
    internal_forces::muscles::IdealizedActuator deepCopyNow(
        idealizedActuator.DeepCopy());
    internal_forces::muscles::IdealizedActuator deepCopyLater;
    deepCopyLater.DeepCopy(idealizedActuator);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, idealizedActuator.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
    }

    internal_forces::muscles::Characteristics charac(
        idealizedActuator.characteristics());
    charac.setPennationAngle(0.523599);
    {
      SCALAR_TO_DOUBLE(
          pennationAngle, idealizedActuator.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    idealizedActuator.updateOrientations(updatedModel, Q, Qdot);

    internal_forces::muscles::IdealizedActuator shallowCopy(idealizedActuator);
    internal_forces::muscles::IdealizedActuator deepCopyNow(
        idealizedActuator.DeepCopy());
    internal_forces::muscles::IdealizedActuator deepCopyLater;
    deepCopyLater.DeepCopy(idealizedActuator);

    {
      SCALAR_TO_DOUBLE(length, idealizedActuator.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 0.066381977535807504, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
    }

    {
      // Change the position of the insertion and pennation angle and compare
      // again (length and insertion in Local)
      internal_forces::muscles::Characteristics charac(
          idealizedActuator.characteristics());
      charac.setPennationAngle(0.523599);
      utils::Vector3d insertion(
          idealizedActuator.position().insertionInLocal());
      insertion.set(0.2, 0.2, 0.2);
      const_cast<internal_forces::muscles::MuscleGeometry&>(
          idealizedActuator.position())
          .setInsertionInLocal(insertion);
      utils::String oldName(insertion.utils::Node::name());
      utils::String newName("MyNewName");
      insertion.setName(newName);
      idealizedActuator.updateOrientations(updatedModel, Q, Qdot);

      {
        SCALAR_TO_DOUBLE(length, idealizedActuator.position().length());
        SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
        SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
        SCALAR_TO_DOUBLE(
            deepCopyLaterLength, deepCopyLater.position().length());
        EXPECT_NEAR(length, 0.45796092269003846, requiredPrecision);
        EXPECT_NEAR(shallowCopyLength, 0.45796092269003846, requiredPrecision);
        EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
        EXPECT_NEAR(
            deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
        EXPECT_EQ(
            idealizedActuator.position().insertionInLocal().utils::Node::name(),
            newName);
        EXPECT_EQ(
            shallowCopy.position().insertionInLocal().utils::Node::name(),
            newName);
        EXPECT_EQ(
            deepCopyNow.position().insertionInLocal().utils::Node::name(),
            oldName);
        EXPECT_EQ(
            deepCopyLater.position().insertionInLocal().utils::Node::name(),
            oldName);
      }
    }

    {
      // Change the position giving an actual vector
      utils::Vector3d newPosition(1, 2, 3);
      utils::String oldName("MyNewName");
      utils::String newName("MyNewNewName");
      rigidbody::NodeSegment newNode(
          newPosition, newName, "", true, true, "", 0);
      {
        const_cast<internal_forces::muscles::MuscleGeometry&>(
            idealizedActuator.position())
            .setOrigin(newPosition);
        const_cast<internal_forces::muscles::MuscleGeometry&>(
            idealizedActuator.position())
            .setInsertionInLocal(newPosition);
        const utils::Vector3d& origin =
            idealizedActuator.position().originInLocal();
        const utils::Vector3d& insertion =
            idealizedActuator.position().insertionInLocal();
        EXPECT_STREQ(origin.utils::Node::name().c_str(), "TRImed_origin");
        EXPECT_STREQ(insertion.utils::Node::name().c_str(), oldName.c_str());
      }
      {
        const_cast<internal_forces::muscles::MuscleGeometry&>(
            idealizedActuator.position())
            .setOrigin(newNode);
        const_cast<internal_forces::muscles::MuscleGeometry&>(
            idealizedActuator.position())
            .setInsertionInLocal(newNode);
        const utils::Vector3d& origin =
            idealizedActuator.position().originInLocal();
        const utils::Vector3d& insertion =
            idealizedActuator.position().insertionInLocal();
        EXPECT_STREQ(origin.utils::Node::name().c_str(), newName.c_str());
        EXPECT_STREQ(insertion.utils::Node::name().c_str(), newName.c_str());
      }
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::IdealizedActuator idealizedActuator(
        model.muscleGroup(muscleGroupForIdealizedActuator)
            .muscle(muscleForIdealizedActuator));

    internal_forces::muscles::IdealizedActuator shallowCopy(idealizedActuator);
    internal_forces::muscles::IdealizedActuator deepCopyNow(
        idealizedActuator.DeepCopy());
    internal_forces::muscles::IdealizedActuator deepCopyLater;
    deepCopyLater.DeepCopy(idealizedActuator);

    {
      SCALAR_TO_DOUBLE(excitation, idealizedActuator.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    idealizedActuator.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(excitation, idealizedActuator.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

static size_t muscleGroupForHillType(1);
static size_t muscleForHillType(1);

TEST(hillType, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType hillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));
    hillType.setName("newName");
    EXPECT_STREQ(hillType.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType hillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillType.updateOrientations(updatedModel, Q, Qdot);
    static double activationEmgForHillTypeTest(1);
    internal_forces::muscles::StateDynamics emg(
        0, activationEmgForHillTypeTest);

    SCALAR_TO_DOUBLE(flce, hillType.FlCE(emg));
    SCALAR_TO_DOUBLE(flpe, hillType.FlPE());
    SCALAR_TO_DOUBLE(fvce, hillType.FvCE());
    SCALAR_TO_DOUBLE(damping, hillType.damping());
    EXPECT_NEAR(flce, 0.67988981401208015, requiredPrecision);
    EXPECT_NEAR(flpe, 0.00010445169885884543, requiredPrecision);
    EXPECT_NEAR(fvce, 1.000886825333013, requiredPrecision);
    EXPECT_NEAR(damping, 0.00019534599393617336, requiredPrecision);

    // with damping
    internal_forces::muscles::Characteristics charac(
        hillType.characteristics());
    SCALAR_TO_DOUBLE(forceDamped, hillType.force(emg));
    EXPECT_NEAR(forceDamped, 419.78610578875896, requiredPrecision);
    EXPECT_EQ(charac.useDamping(), 1);

    // without damping
    charac.setUseDamping(false);
    SCALAR_TO_DOUBLE(force, hillType.force(emg));
    EXPECT_NEAR(force, 419.66565274700974, requiredPrecision);
    EXPECT_EQ(charac.useDamping(), 0);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType originalHillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));

    internal_forces::muscles::HillType newHillType(
        "newName",
        originalHillType.position(),
        originalHillType.characteristics());

    EXPECT_STREQ(newHillType.name().c_str(), "newName");
    EXPECT_EQ(newHillType.type(), internal_forces::muscles::MUSCLE_TYPE::HILL);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType originalHillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));

    internal_forces::muscles::HillType newHillType(
        "newName",
        originalHillType.position(),
        originalHillType.characteristics(),
        originalHillType.state());

    EXPECT_STREQ(newHillType.name().c_str(), "newName");
    EXPECT_EQ(newHillType.type(), internal_forces::muscles::MUSCLE_TYPE::HILL);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType originalHillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));

    internal_forces::muscles::HillType newHillType(
        "newName",
        originalHillType.position(),
        originalHillType.characteristics(),
        originalHillType.pathModifier());

    EXPECT_STREQ(newHillType.name().c_str(), "newName");
    EXPECT_EQ(newHillType.type(), internal_forces::muscles::MUSCLE_TYPE::HILL);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType hillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    static double activationEmgForHillTypeTest(1);
    internal_forces::muscles::StateDynamics emg(
        0, activationEmgForHillTypeTest);

    SCALAR_TO_DOUBLE(force, hillType.force(updatedModel, Q, Qdot, emg, 2));
    EXPECT_NEAR(force, 419.78610578875896, requiredPrecision);
  }
}

TEST(hillType, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType hillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));
    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::HillType shallowCopy(hillType);
    internal_forces::muscles::HillType deepCopyNow(hillType.DeepCopy());
    internal_forces::muscles::HillType deepCopyLater;
    deepCopyLater.DeepCopy(hillType);

    utils::String originalName(hillType.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    hillType.setName(newName);
    EXPECT_STREQ(hillType.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType hillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));

    // copies of the Hill Type muscle
    internal_forces::muscles::HillType shallowCopy(hillType);
    internal_forces::muscles::HillType deepCopyNow(hillType.DeepCopy());
    internal_forces::muscles::HillType deepCopyLater;
    deepCopyLater.DeepCopy(hillType);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
    }

    internal_forces::muscles::Characteristics charac(
        hillType.characteristics());
    charac.setPennationAngle(0.523599);
    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType hillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillType.updateOrientations(updatedModel, Q);

    internal_forces::muscles::HillType shallowCopy(hillType);
    internal_forces::muscles::HillType deepCopyNow(hillType.DeepCopy());
    internal_forces::muscles::HillType deepCopyLater;
    deepCopyLater.DeepCopy(hillType);

    {
      SCALAR_TO_DOUBLE(length, hillType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 0.066381977535807504, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
    }

    // Change the position of the insertion and pennation angle and compare
    // again (length and insertion in Local)
    internal_forces::muscles::Characteristics charac(
        hillType.characteristics());
    charac.setPennationAngle(0.523599);
    utils::Vector3d insertion(hillType.position().insertionInLocal());
    insertion.set(0.5, 0.6, 0.7);
    const_cast<internal_forces::muscles::MuscleGeometry&>(hillType.position())
        .setInsertionInLocal(insertion);
    utils::String oldName(insertion.utils::Node::name());
    utils::String newName("MyNewName");
    insertion.setName(newName);
    hillType.updateOrientations(updatedModel, Q, Qdot);

    {
      SCALAR_TO_DOUBLE(length, hillType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 1.2655089931238894, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 1.2655089931238894, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
      EXPECT_EQ(
          hillType.position().insertionInLocal().utils::Node::name(), newName);
      EXPECT_EQ(
          shallowCopy.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          deepCopyNow.position().insertionInLocal().utils::Node::name(),
          oldName);
      EXPECT_EQ(
          deepCopyLater.position().insertionInLocal().utils::Node::name(),
          oldName);
    }
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillType hillType(
        model.muscleGroup(muscleGroupForHillType).muscle(muscleForHillType));

    internal_forces::muscles::HillType shallowCopy(hillType);
    internal_forces::muscles::HillType deepCopyNow(hillType.DeepCopy());
    internal_forces::muscles::HillType deepCopyLater;
    deepCopyLater.DeepCopy(hillType);

    {
      SCALAR_TO_DOUBLE(excitation, hillType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    hillType.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(excitation, hillType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

static size_t muscleGroupForHillThelenType(1);
static size_t muscleForHillThelenType(0);

TEST(hillThelenType, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    hillThelenType.setName("newName");
    EXPECT_STREQ(hillThelenType.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillThelenType.updateOrientations(updatedModel, Q, Qdot);
    static double activationEmgForHillTypeTest(1.0);
    internal_forces::muscles::StateDynamics emg(
        0, activationEmgForHillTypeTest);

    SCALAR_TO_DOUBLE(flce, hillThelenType.FlCE(emg));
    SCALAR_TO_DOUBLE(flpe, hillThelenType.FlPE());
    SCALAR_TO_DOUBLE(fvce, hillThelenType.FvCE());
    SCALAR_TO_DOUBLE(damping, hillThelenType.damping());
    SCALAR_TO_DOUBLE(force, hillThelenType.force(emg));
    EXPECT_NEAR(flce, 0.73689336678824058, requiredPrecision);
    EXPECT_NEAR(flpe, 0, requiredPrecision);
    EXPECT_NEAR(fvce, 1.0189186522393461, requiredPrecision);
    EXPECT_NEAR(damping, 0.00019534599393617336, requiredPrecision);
    EXPECT_NEAR(force, 462.97487366718485, requiredPrecision);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenType hillThelenTypeNew(
        "newName", hillThelenType.position(), hillThelenType.characteristics());

    EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenType hillThelenTypeNew(
        "newName",
        hillThelenType.position(),
        hillThelenType.characteristics(),
        hillThelenType.state());

    EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenType hillThelenTypeNew(
        "newName",
        hillThelenType.position(),
        hillThelenType.characteristics(),
        hillThelenType.pathModifier());

    EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN);
  }
}

TEST(hillThelenType, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::HillThelenType shallowCopy(hillThelenType);
    internal_forces::muscles::HillThelenType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    utils::String originalName(hillThelenType.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    hillThelenType.setName(newName);
    EXPECT_STREQ(hillThelenType.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenType shallowCopy(hillThelenType);
    internal_forces::muscles::HillThelenType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillThelenType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
    }

    internal_forces::muscles::Characteristics charac(
        hillThelenType.characteristics());
    charac.setPennationAngle(0.523599);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillThelenType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillThelenType.updateOrientations(updatedModel, Q);

    internal_forces::muscles::HillThelenType shallowCopy(hillThelenType);
    internal_forces::muscles::HillThelenType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    {
      SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.071618646132835737, requiredPrecision);
    }

    // Change the position of the insertion and pennation angle and compare
    // again (length and insertion in Local)
    internal_forces::muscles::Characteristics charac(
        hillThelenType.characteristics());
    charac.setPennationAngle(0.523599);
    utils::Vector3d insertion(hillThelenType.position().insertionInLocal());
    insertion.set(0.5, 0.6, 0.7);
    const_cast<internal_forces::muscles::MuscleGeometry&>(
        hillThelenType.position())
        .setInsertionInLocal(insertion);
    utils::String oldName(insertion.utils::Node::name());
    utils::String newName("MyNewName");
    insertion.setName(newName);
    hillThelenType.updateOrientations(updatedModel, Q, Qdot);

    {
      SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 1.2714813320027956, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 1.2714813320027956, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.071618646132835737, requiredPrecision);
      EXPECT_EQ(
          hillThelenType.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          shallowCopy.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          deepCopyNow.position().insertionInLocal().utils::Node::name(),
          oldName);
      EXPECT_EQ(
          deepCopyLater.position().insertionInLocal().utils::Node::name(),
          oldName);
    }
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenType shallowCopy(hillThelenType);
    internal_forces::muscles::HillThelenType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    {
      SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    hillThelenType.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

TEST(hillThelenTypeActive, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    hillThelenType.setName("newName");
    EXPECT_STREQ(hillThelenType.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillThelenType.updateOrientations(updatedModel, Q, Qdot);
    static double activationEmgForHillTypeTest(1.0);
    internal_forces::muscles::StateDynamics emg(
        0, activationEmgForHillTypeTest);

    SCALAR_TO_DOUBLE(flce, hillThelenType.FlCE(emg));
    SCALAR_TO_DOUBLE(flpe, hillThelenType.FlPE());
    SCALAR_TO_DOUBLE(fvce, hillThelenType.FvCE());
    SCALAR_TO_DOUBLE(damping, hillThelenType.damping());
    SCALAR_TO_DOUBLE(force, hillThelenType.force(emg));
    EXPECT_NEAR(flce, 0.73689336678824058, requiredPrecision);
    EXPECT_NEAR(flpe, 0., requiredPrecision);
    EXPECT_NEAR(fvce, 1.0189186522393461, requiredPrecision);
    EXPECT_NEAR(damping, 0., requiredPrecision);
    EXPECT_NEAR(force, 462.97487366718485, requiredPrecision);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenActiveOnlyType hillThelenTypeNew(
        "newName", hillThelenType.position(), hillThelenType.characteristics());

    EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenActiveOnlyType hillThelenTypeNew(
        "newName",
        hillThelenType.position(),
        hillThelenType.characteristics(),
        hillThelenType.state());

    EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenActiveOnlyType hillThelenTypeNew(
        "newName",
        hillThelenType.position(),
        hillThelenType.characteristics(),
        hillThelenType.pathModifier());

    EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE);
  }
}

TEST(hillThelenActiveType, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::HillThelenActiveOnlyType shallowCopy(
        hillThelenType);
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    utils::String originalName(hillThelenType.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    hillThelenType.setName(newName);
    EXPECT_STREQ(hillThelenType.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenActiveOnlyType shallowCopy(
        hillThelenType);
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillThelenType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
    }

    internal_forces::muscles::Characteristics charac(
        hillThelenType.characteristics());
    charac.setPennationAngle(0.523599);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillThelenType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillThelenType.updateOrientations(updatedModel, Q);

    internal_forces::muscles::HillThelenActiveOnlyType shallowCopy(
        hillThelenType);
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    {
      SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.071618646132835737, requiredPrecision);
    }

    // Change the position of the insertion and pennation angle and compare
    // again (length and insertion in Local)
    internal_forces::muscles::Characteristics charac(
        hillThelenType.characteristics());
    charac.setPennationAngle(0.523599);
    utils::Vector3d insertion(hillThelenType.position().insertionInLocal());
    insertion.set(0.5, 0.6, 0.7);
    const_cast<internal_forces::muscles::MuscleGeometry&>(
        hillThelenType.position())
        .setInsertionInLocal(insertion);
    utils::String oldName(insertion.utils::Node::name());
    utils::String newName("MyNewName");
    insertion.setName(newName);
    hillThelenType.updateOrientations(updatedModel, Q, Qdot);

    {
      SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 1.2714813320027956, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 1.2714813320027956, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.071618646132835737, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.071618646132835737, requiredPrecision);
      EXPECT_EQ(
          hillThelenType.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          shallowCopy.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          deepCopyNow.position().insertionInLocal().utils::Node::name(),
          oldName);
      EXPECT_EQ(
          deepCopyLater.position().insertionInLocal().utils::Node::name(),
          oldName);
    }
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenActiveOnlyType hillThelenType(
        model.muscleGroup(muscleGroupForHillThelenType)
            .muscle(muscleForHillThelenType));

    internal_forces::muscles::HillThelenActiveOnlyType shallowCopy(
        hillThelenType);
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyNow(
        hillThelenType.DeepCopy());
    internal_forces::muscles::HillThelenActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenType);

    {
      SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    hillThelenType.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

static size_t muscleGroupFordeGrooteType(0);
static size_t muscleFordeGrooteType(2);
TEST(hillDeGrooteTypeActive, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    hillDeGrooteType.setName("newName");
    EXPECT_STREQ(hillDeGrooteType.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillDeGrooteType.updateOrientations(updatedModel, Q, Qdot);

    static double activationEmgForHillTypeTest(1.0);
    internal_forces::muscles::StateDynamics emg(
        0, activationEmgForHillTypeTest);

    SCALAR_TO_DOUBLE(flce, hillDeGrooteType.FlCE(emg));
    SCALAR_TO_DOUBLE(flpe, hillDeGrooteType.FlPE());
    SCALAR_TO_DOUBLE(fvce, hillDeGrooteType.FvCE());
    SCALAR_TO_DOUBLE(damping, hillDeGrooteType.damping());
    SCALAR_TO_DOUBLE(force, hillDeGrooteType.force(emg));
    EXPECT_NEAR(flce, 0.92953865278129677, requiredPrecision);
    EXPECT_NEAR(flpe, 0., requiredPrecision);
    EXPECT_NEAR(fvce, 0.99966505861454702, requiredPrecision);
    EXPECT_NEAR(damping, 0., requiredPrecision);
    EXPECT_NEAR(force, 404.734247935057, requiredPrecision);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteTypeNew(
        "newName",
        hillDeGrooteType.position(),
        hillDeGrooteType.characteristics());

    EXPECT_STREQ(hillDeGrooteTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteTypeNew(
        "newName",
        hillDeGrooteType.position(),
        hillDeGrooteType.characteristics(),
        hillDeGrooteType.state());

    EXPECT_STREQ(hillDeGrooteTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteTypeNew(
        "newName",
        hillDeGrooteType.position(),
        hillDeGrooteType.characteristics(),
        hillDeGrooteType.pathModifier());

    EXPECT_STREQ(hillDeGrooteTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE);
  }
}

TEST(hillDeGrooteActiveType, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::HillDeGrooteActiveOnlyType shallowCopy(
        hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    utils::String originalName(hillDeGrooteType.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    hillDeGrooteType.setName(newName);
    EXPECT_STREQ(hillDeGrooteType.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteActiveOnlyType shallowCopy(
        hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillDeGrooteType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0, requiredPrecision);
    }

    internal_forces::muscles::Characteristics charac(
        hillDeGrooteType.characteristics());
    charac.setPennationAngle(0.523599);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillDeGrooteType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0, requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillDeGrooteType.updateOrientations(updatedModel, Q);

    internal_forces::muscles::HillDeGrooteActiveOnlyType shallowCopy(
        hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    {
      SCALAR_TO_DOUBLE(length, hillDeGrooteType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.14782350513656897, requiredPrecision);
    }

    // Change the position of the insertion and pennation angle and compare
    // again (length and insertion in Local)
    internal_forces::muscles::Characteristics charac(
        hillDeGrooteType.characteristics());
    charac.setPennationAngle(0.523599);
    utils::Vector3d insertion(hillDeGrooteType.position().insertionInLocal());
    insertion.set(0.5, 0.6, 0.7);
    const_cast<internal_forces::muscles::MuscleGeometry&>(
        hillDeGrooteType.position())
        .setInsertionInLocal(insertion);
    utils::String oldName(insertion.utils::Node::name());
    utils::String newName("MyNewName");
    insertion.setName(newName);
    hillDeGrooteType.updateOrientations(updatedModel, Q, Qdot);

    {
      SCALAR_TO_DOUBLE(length, hillDeGrooteType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 1.1030846548794651, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 1.1030846548794651, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.14782350513656897, requiredPrecision);
      EXPECT_EQ(
          hillDeGrooteType.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          shallowCopy.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          deepCopyNow.position().insertionInLocal().utils::Node::name(),
          oldName);
      EXPECT_EQ(
          deepCopyLater.position().insertionInLocal().utils::Node::name(),
          oldName);
    }
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteActiveOnlyType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteActiveOnlyType shallowCopy(
        hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteActiveOnlyType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    {
      SCALAR_TO_DOUBLE(excitation, hillDeGrooteType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    hillDeGrooteType.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(excitation, hillDeGrooteType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

static size_t muscleGroupForhillDeGrooteTypeFatigable(1);
static size_t muscleForhillDeGrooteTypeFatigable(1);
TEST(hillDeGrooteTypeFatigable, unitTest) {
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;
    hillDeGrooteTypeFatigable.setName("newName");
    EXPECT_STREQ(hillDeGrooteTypeFatigable.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    EXPECT_THROW(
        internal_forces::muscles::HillDeGrooteTypeFatigable
            hillDeGrooteTypeFatigable(
                model.muscleGroup(muscleGroupForhillDeGrooteTypeFatigable)
                    .muscle(muscleForhillDeGrooteTypeFatigable)),
        std::bad_cast);
  }
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigableNew(
            "newName",
            hillDeGrooteTypeFatigable.position(),
            hillDeGrooteTypeFatigable.characteristics(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    EXPECT_STREQ(hillDeGrooteTypeFatigableNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE);
  }

  {
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigableNew(
            "newName",
            hillDeGrooteTypeFatigable.position(),
            hillDeGrooteTypeFatigable.characteristics(),
            hillDeGrooteTypeFatigable.state());

    EXPECT_STREQ(hillDeGrooteTypeFatigableNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE);
  }
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigableNew(
            "newName",
            hillDeGrooteTypeFatigable.position(),
            hillDeGrooteTypeFatigable.characteristics(),
            hillDeGrooteTypeFatigable.pathModifier(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    EXPECT_STREQ(hillDeGrooteTypeFatigableNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE);
  }
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigableNew(
            "nameMuscle",
            hillDeGrooteTypeFatigable.position(),
            hillDeGrooteTypeFatigable.characteristics(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    EXPECT_EQ(
        hillDeGrooteTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE);
  }
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigableNew(
            "nameMuscle",
            hillDeGrooteTypeFatigable.position(),
            hillDeGrooteTypeFatigable.characteristics(),
            hillDeGrooteTypeFatigable.pathModifier(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    EXPECT_EQ(
        hillDeGrooteTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE);
  }
}
TEST(hillDeGrooteTypeFatigable, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::HillDeGrooteTypeFatigable shallowCopy(
        hillDeGrooteTypeFatigable);
    internal_forces::muscles::HillDeGrooteTypeFatigable deepCopyNow(
        hillDeGrooteTypeFatigable.DeepCopy());
    internal_forces::muscles::HillDeGrooteTypeFatigable deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteTypeFatigable);

    utils::String originalName(hillDeGrooteTypeFatigable.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    hillDeGrooteTypeFatigable.setName(newName);
    EXPECT_STREQ(hillDeGrooteTypeFatigable.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    internal_forces::muscles::HillDeGrooteTypeFatigable shallowCopy(
        hillDeGrooteTypeFatigable);
    internal_forces::muscles::HillDeGrooteTypeFatigable deepCopyNow(
        hillDeGrooteTypeFatigable.DeepCopy());
    internal_forces::muscles::HillDeGrooteTypeFatigable deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteTypeFatigable);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle,
          hillDeGrooteTypeFatigable.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0., requiredPrecision);
    }
    internal_forces::muscles::Characteristics charac(
        hillDeGrooteTypeFatigable.characteristics());
    charac.setPennationAngle(0.523599);
    {
      SCALAR_TO_DOUBLE(
          pennationAngle,
          hillDeGrooteTypeFatigable.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0., requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteTypeFatigable
        hillDeGrooteTypeFatigable;

    internal_forces::muscles::HillDeGrooteTypeFatigable shallowCopy(
        hillDeGrooteTypeFatigable);
    internal_forces::muscles::HillDeGrooteTypeFatigable deepCopyNow(
        hillDeGrooteTypeFatigable.DeepCopy());
    internal_forces::muscles::HillDeGrooteTypeFatigable deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteTypeFatigable);

    {
      SCALAR_TO_DOUBLE(
          excitation, hillDeGrooteTypeFatigable.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    hillDeGrooteTypeFatigable.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(
          excitation, hillDeGrooteTypeFatigable.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

TEST(hillDeGrooteType, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    hillDeGrooteType.setName("newName");
    EXPECT_STREQ(hillDeGrooteType.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillDeGrooteType.updateOrientations(updatedModel, Q, Qdot);

    static double activationEmgForHillTypeTest(1.0);
    internal_forces::muscles::StateDynamics emg(
        0, activationEmgForHillTypeTest);

    SCALAR_TO_DOUBLE(flce, hillDeGrooteType.FlCE(emg));
    SCALAR_TO_DOUBLE(flpe, hillDeGrooteType.FlPE());
    SCALAR_TO_DOUBLE(fvce, hillDeGrooteType.FvCE());
    SCALAR_TO_DOUBLE(damping, hillDeGrooteType.damping());
    SCALAR_TO_DOUBLE(force, hillDeGrooteType.force(emg));
    EXPECT_NEAR(flce, 0.92953865278129677, requiredPrecision);
    EXPECT_NEAR(flpe, 0.022596966416076593, requiredPrecision);
    EXPECT_NEAR(fvce, 0.99966505861454702, requiredPrecision);
    EXPECT_NEAR(damping, 0, requiredPrecision);
    EXPECT_NEAR(force, 414.57658262724328, requiredPrecision);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteType hillDeGrooteTypeNew(
        "newName",
        hillDeGrooteType.position(),
        hillDeGrooteType.characteristics());

    EXPECT_STREQ(hillDeGrooteTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteType hillDeGrooteTypeNew(
        "newName",
        hillDeGrooteType.position(),
        hillDeGrooteType.characteristics(),
        hillDeGrooteType.state());

    EXPECT_STREQ(hillDeGrooteTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteType hillDeGrooteTypeNew(
        "newName",
        hillDeGrooteType.position(),
        hillDeGrooteType.characteristics(),
        hillDeGrooteType.pathModifier());

    EXPECT_STREQ(hillDeGrooteTypeNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillDeGrooteTypeNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE);
  }
}

TEST(hillDeGrooteType, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::HillDeGrooteType shallowCopy(hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    utils::String originalName(hillDeGrooteType.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    hillDeGrooteType.setName(newName);
    EXPECT_STREQ(hillDeGrooteType.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteType shallowCopy(hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillDeGrooteType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0, requiredPrecision);
    }

    internal_forces::muscles::Characteristics charac(
        hillDeGrooteType.characteristics());
    charac.setPennationAngle(0.523599);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle, hillDeGrooteType.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0, requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));
    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes() / 10;
    Qdot = Qdot.setOnes() / 10;
    auto updatedModel = model.UpdateKinematicsCustom(&Q, &Qdot);

    hillDeGrooteType.updateOrientations(updatedModel, Q);

    internal_forces::muscles::HillDeGrooteType shallowCopy(hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    {
      SCALAR_TO_DOUBLE(length, hillDeGrooteType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.14782350513656897, requiredPrecision);
    }

    // Change the position of the insertion and pennation angle and compare
    // again (length and insertion in Local)
    internal_forces::muscles::Characteristics charac(
        hillDeGrooteType.characteristics());
    charac.setPennationAngle(0.523599);
    utils::Vector3d insertion(hillDeGrooteType.position().insertionInLocal());
    insertion.set(0.5, 0.6, 0.7);
    const_cast<internal_forces::muscles::MuscleGeometry&>(
        hillDeGrooteType.position())
        .setInsertionInLocal(insertion);
    utils::String oldName(insertion.utils::Node::name());
    utils::String newName("MyNewName");
    insertion.setName(newName);
    hillDeGrooteType.updateOrientations(updatedModel, Q, Qdot);

    {
      SCALAR_TO_DOUBLE(length, hillDeGrooteType.position().length());
      SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
      SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
      SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
      EXPECT_NEAR(length, 1.1030846548794651, requiredPrecision);
      EXPECT_NEAR(shallowCopyLength, 1.1030846548794651, requiredPrecision);
      EXPECT_NEAR(deepCopyNowLength, 0.14782350513656897, requiredPrecision);
      EXPECT_NEAR(deepCopyLaterLength, 0.14782350513656897, requiredPrecision);
      EXPECT_EQ(
          hillDeGrooteType.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          shallowCopy.position().insertionInLocal().utils::Node::name(),
          newName);
      EXPECT_EQ(
          deepCopyNow.position().insertionInLocal().utils::Node::name(),
          oldName);
      EXPECT_EQ(
          deepCopyLater.position().insertionInLocal().utils::Node::name(),
          oldName);
    }
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillDeGrooteType hillDeGrooteType(
        model.muscleGroup(muscleGroupFordeGrooteType)
            .muscle(muscleFordeGrooteType));

    internal_forces::muscles::HillDeGrooteType shallowCopy(hillDeGrooteType);
    internal_forces::muscles::HillDeGrooteType deepCopyNow(
        hillDeGrooteType.DeepCopy());
    internal_forces::muscles::HillDeGrooteType deepCopyLater;
    deepCopyLater.DeepCopy(hillDeGrooteType);

    {
      SCALAR_TO_DOUBLE(excitation, hillDeGrooteType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    hillDeGrooteType.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(excitation, hillDeGrooteType.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

TEST(DynamicState, Normal) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::StateDynamics state(0.8, 0.5);

    const internal_forces::muscles::Muscle& m(model.muscle(0));
    SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
    EXPECT_NEAR(actDot, 24.0, requiredPrecision);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::StateDynamics state(0.3, 0.5);

    const internal_forces::muscles::Muscle& m(model.muscle(0));
    SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
    EXPECT_NEAR(actDot, -6.25, requiredPrecision);
  }
}

TEST(DynamicState, Buchanan) {
  {
    Model model(modelPathForBuchananDynamics);
    static_cast<internal_forces::muscles::StateDynamicsBuchanan&>(
        model.muscles()[2]->state())
        .shapeFactor(5.5);

    // Test shape factor changes
    utils::Scalar shapeFactor0 =
        static_cast<internal_forces::muscles::StateDynamicsBuchanan&>(
            model.muscles()[0]->state())
            .shapeFactor();
    SCALAR_TO_DOUBLE(shapeFactor0_double, shapeFactor0);
    EXPECT_NEAR(shapeFactor0_double, -3, requiredPrecision);

    utils::Scalar shapeFactor1 =
        static_cast<internal_forces::muscles::StateDynamicsBuchanan&>(
            model.muscles()[1]->state())
            .shapeFactor();
    SCALAR_TO_DOUBLE(shapeFactor1_double, shapeFactor1);
    EXPECT_NEAR(shapeFactor1_double, 10, requiredPrecision);

    utils::Scalar shapeFactor2 =
        static_cast<internal_forces::muscles::StateDynamicsBuchanan&>(
            model.muscles()[2]->state())
            .shapeFactor();
    SCALAR_TO_DOUBLE(shapeFactor2_double, shapeFactor2);
    EXPECT_NEAR(shapeFactor2_double, 5.5, requiredPrecision);

    internal_forces::muscles::StateDynamics state(0.8, 0.5);

    const internal_forces::muscles::Muscle& m(model.muscle(0));
    SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
    EXPECT_NEAR(actDot, -7.592740648890816, requiredPrecision);
  }
  {
    Model model(modelPathForBuchananDynamics);
    internal_forces::muscles::StateDynamics state(0.3, 0.5);

    const internal_forces::muscles::Muscle& m(model.muscle(0));
    SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
    EXPECT_NEAR(actDot, -11.656766195499843, requiredPrecision);
  }
}

TEST(DynamicState, DeGroote) {
  {
    Model model(modelPathForDeGrooteDynamics);
    internal_forces::muscles::StateDynamics state(0.8, 0.5);

    const internal_forces::muscles::Muscle& m(model.muscle(0));
    SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
    EXPECT_NEAR(actDot, 16.906809211183873, requiredPrecision);
  }
  {
    Model model(modelPathForDeGrooteDynamics);
    internal_forces::muscles::StateDynamics state(0.3, 0.5);

    const internal_forces::muscles::Muscle& m(model.muscle(0));
    SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
    EXPECT_NEAR(actDot, -11.027512997920336, requiredPrecision);
  }
}

static size_t muscleGroupForHillThelenTypeFatigable(1);
static size_t muscleForHillThelenTypeFatigable(1);

TEST(hillThelenTypeFatigable, unitTest) {
  {
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;
    hillThelenTypeFatigable.setName("newName");
    EXPECT_STREQ(hillThelenTypeFatigable.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    EXPECT_THROW(
        internal_forces::muscles::HillThelenTypeFatigable
            hillThelenTypeFatigable(
                model.muscleGroup(muscleGroupForHillThelenTypeFatigable)
                    .muscle(muscleForHillThelenTypeFatigable)),
        std::bad_cast);
  }
  {
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    internal_forces::muscles::HillThelenTypeFatigable
        hillThelenTypeFatigableNew(
            "newName",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    EXPECT_STREQ(hillThelenTypeFatigableNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
  }

  {
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    internal_forces::muscles::HillThelenTypeFatigable
        hillThelenTypeFatigableNew(
            "newName",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            hillThelenTypeFatigable.state());

    EXPECT_STREQ(hillThelenTypeFatigableNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
  }
  {
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    internal_forces::muscles::HillThelenTypeFatigable
        hillThelenTypeFatigableNew(
            "newName",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            hillThelenTypeFatigable.pathModifier(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    EXPECT_STREQ(hillThelenTypeFatigableNew.name().c_str(), "newName");
    EXPECT_EQ(
        hillThelenTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
  }
  {
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    internal_forces::muscles::HillThelenTypeFatigable
        hillThelenTypeFatigableNew(
            "nameMuscle",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    EXPECT_EQ(
        hillThelenTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
  }
  {
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    internal_forces::muscles::HillThelenTypeFatigable
        hillThelenTypeFatigableNew(
            "nameMuscle",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            hillThelenTypeFatigable.pathModifier(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    EXPECT_EQ(
        hillThelenTypeFatigableNew.type(),
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
  }
}

TEST(hillThelenTypeFatigable, copy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes() / 10;

    internal_forces::muscles::HillThelenTypeFatigable shallowCopy(
        hillThelenTypeFatigable);
    internal_forces::muscles::HillThelenTypeFatigable deepCopyNow(
        hillThelenTypeFatigable.DeepCopy());
    internal_forces::muscles::HillThelenTypeFatigable deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenTypeFatigable);

    utils::String originalName(hillThelenTypeFatigable.name());
    EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

    utils::String newName("MyNewMuscleName");
    hillThelenTypeFatigable.setName(newName);
    EXPECT_STREQ(hillThelenTypeFatigable.name().c_str(), newName.c_str());
    EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
    EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
    EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    internal_forces::muscles::HillThelenTypeFatigable shallowCopy(
        hillThelenTypeFatigable);
    internal_forces::muscles::HillThelenTypeFatigable deepCopyNow(
        hillThelenTypeFatigable.DeepCopy());
    internal_forces::muscles::HillThelenTypeFatigable deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenTypeFatigable);

    {
      SCALAR_TO_DOUBLE(
          pennationAngle,
          hillThelenTypeFatigable.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0., requiredPrecision);
    }
    internal_forces::muscles::Characteristics charac(
        hillThelenTypeFatigable.characteristics());
    charac.setPennationAngle(0.523599);
    {
      SCALAR_TO_DOUBLE(
          pennationAngle,
          hillThelenTypeFatigable.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          shallowCopyPennationAngle,
          shallowCopy.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyNowPennationAngle,
          deepCopyNow.characteristics().pennationAngle());
      SCALAR_TO_DOUBLE(
          deepCopyLaterPennationAngle,
          deepCopyLater.characteristics().pennationAngle());
      EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
      EXPECT_NEAR(deepCopyNowPennationAngle, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterPennationAngle, 0., requiredPrecision);
    }
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

    internal_forces::muscles::HillThelenTypeFatigable shallowCopy(
        hillThelenTypeFatigable);
    internal_forces::muscles::HillThelenTypeFatigable deepCopyNow(
        hillThelenTypeFatigable.DeepCopy());
    internal_forces::muscles::HillThelenTypeFatigable deepCopyLater;
    deepCopyLater.DeepCopy(hillThelenTypeFatigable);

    {
      SCALAR_TO_DOUBLE(
          excitation, hillThelenTypeFatigable.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 0., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }

    hillThelenTypeFatigable.state().setExcitation(utils::Scalar(5.));

    {
      SCALAR_TO_DOUBLE(
          excitation, hillThelenTypeFatigable.state().excitation());
      SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
      SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
      SCALAR_TO_DOUBLE(
          deepCopyLaterExcitation, deepCopyLater.state().excitation());
      EXPECT_NEAR(excitation, 5., requiredPrecision);
      EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
      EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
      EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
    }
  }
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(FatigueState, unitTest) {
  internal_forces::muscles::FatigueState fatigueState;
  fatigueState.setState(0.0, 1.0, 0.0);

  SCALAR_TO_DOUBLE(activeFibers, fatigueState.activeFibers());
  SCALAR_TO_DOUBLE(fatiguedFibers, fatigueState.fatiguedFibers());
  SCALAR_TO_DOUBLE(restingFibers, fatigueState.restingFibers());
  EXPECT_EQ(activeFibers, 0.0);
  EXPECT_EQ(fatiguedFibers, 1.);
  EXPECT_EQ(restingFibers, 0.);
}
#endif

TEST(FatigueState, copy) {
  internal_forces::muscles::FatigueState fatigueState;

  internal_forces::muscles::FatigueState shallowCopy(fatigueState);
  internal_forces::muscles::FatigueState deepCopyNow(fatigueState.DeepCopy());
  internal_forces::muscles::FatigueState deepCopyLater;
  deepCopyLater.DeepCopy(fatigueState);

  {
    SCALAR_TO_DOUBLE(fatiguedFibers, fatigueState.fatiguedFibers());
    SCALAR_TO_DOUBLE(shallowCopyFatigued, shallowCopy.fatiguedFibers());
    SCALAR_TO_DOUBLE(deepCopyNowFatigued, deepCopyNow.fatiguedFibers());
    SCALAR_TO_DOUBLE(deepCopyLaterFatigued, deepCopyLater.fatiguedFibers());
    EXPECT_EQ(fatiguedFibers, 0.0);
    EXPECT_EQ(shallowCopyFatigued, 0.0);
    EXPECT_EQ(deepCopyNowFatigued, 0.0);
    EXPECT_EQ(deepCopyLaterFatigued, 0.0);
  }

#ifndef BIORBD_USE_CASADI_MATH
  fatigueState.setState(0.0, 1.0, 0.0);
  // change state
  {
    SCALAR_TO_DOUBLE(fatiguedFibers, fatigueState.fatiguedFibers());
    SCALAR_TO_DOUBLE(shallowCopyFatigued, shallowCopy.fatiguedFibers());
    SCALAR_TO_DOUBLE(deepCopyNowFatigued, deepCopyNow.fatiguedFibers());
    SCALAR_TO_DOUBLE(deepCopyLaterFatigued, deepCopyLater.fatiguedFibers());
    EXPECT_EQ(fatiguedFibers, 1.0);
    EXPECT_EQ(shallowCopyFatigued, 1.0);
    EXPECT_EQ(deepCopyNowFatigued, 0.0);
    EXPECT_EQ(deepCopyLaterFatigued, 0.0);
  }
#endif
}

static std::string modelPathForXiaDerivativeTest("models/arm26.bioMod");
static size_t muscleGroupForXiaDerivativeTest(0);
static size_t muscleForXiaDerivativeTest(0);
#ifndef BIORBD_USE_CASADI_MATH
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
#endif

TEST(FatigueDynamiqueState, DeepCopy) {
  // Prepare the model
  Model model(modelPathForXiaDerivativeTest);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  Q.setZero();
  Qdot.setZero();
  model.updateMuscles(Q, Qdot, true);

  {
    internal_forces::muscles::HillThelenTypeFatigable muscle(
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest));
    internal_forces::muscles::FatigueDynamicStateXia fatigueDynamicStateXia;
    // Sanity check for the fatigue model
    EXPECT_EQ(
        fatigueDynamicStateXia.getType(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    // Initial value sanity check
    {
      SCALAR_TO_DOUBLE(val, fatigueDynamicStateXia.activeFibersDot());
      EXPECT_NEAR(val, 0, requiredPrecision);
    }

#ifndef BIORBD_USE_CASADI_MATH
    // Apply the derivative
    internal_forces::muscles::StateDynamics emg(
        0,
        activationEmgForXiaDerivativeTest);  // Set target
    fatigueDynamicStateXia.setState(
        currentActiveFibersForXiaDerivativeTest,
        currentFatiguedFibersForXiaDerivativeTest,
        currentRestingFibersForXiaDerivativeTest);
    fatigueDynamicStateXia.timeDerivativeState(
        emg,
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest)
            .characteristics());

    // Check the values
    EXPECT_NEAR(
        fatigueDynamicStateXia.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);

    // Make copies
    internal_forces::muscles::FatigueDynamicStateXia shallowCopy(
        fatigueDynamicStateXia);
    internal_forces::muscles::FatigueDynamicStateXia deepCopyNow(
        fatigueDynamicStateXia.DeepCopy());
    internal_forces::muscles::FatigueDynamicStateXia deepCopyLater;
    deepCopyLater.DeepCopy(fatigueDynamicStateXia);

    // Check the values
    EXPECT_NEAR(
        shallowCopy.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        deepCopyNow.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        deepCopyLater.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);

    // Change state to change Fibers Dots
    fatigueDynamicStateXia.setState(0.5, 0.5, 0.0);
    fatigueDynamicStateXia.timeDerivativeState(
        emg,
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest)
            .characteristics());

    // Check the new values
    EXPECT_NEAR(
        fatigueDynamicStateXia.activeFibersDot(),
        -0.0050000000000000001,
        requiredPrecision);
    EXPECT_NEAR(
        shallowCopy.activeFibersDot(),
        -0.0050000000000000001,
        requiredPrecision);
    EXPECT_NEAR(
        deepCopyNow.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        deepCopyLater.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
#endif
  }
}

TEST(FatigueParameters, unitTest) {
  internal_forces::muscles::FatigueParameters fatigueParameters;
  fatigueParameters.setFatigueRate(25.0);
  SCALAR_TO_DOUBLE(fatigueRate, fatigueParameters.fatigueRate());
  EXPECT_EQ(fatigueRate, 25.0);

  fatigueParameters.setRecoveryRate(10.0);
  SCALAR_TO_DOUBLE(recoveryRate, fatigueParameters.recoveryRate());
  EXPECT_EQ(recoveryRate, 10.0);

  fatigueParameters.setDevelopFactor(3.0);
  SCALAR_TO_DOUBLE(developFactor, fatigueParameters.developFactor());
  EXPECT_EQ(developFactor, 3.0);

  fatigueParameters.setRecoveryFactor(2.0);
  SCALAR_TO_DOUBLE(recoveryFactor, fatigueParameters.recoveryFactor());
  EXPECT_EQ(recoveryFactor, 2.0);
}

TEST(FatigueParemeters, copy) {
  internal_forces::muscles::FatigueParameters fatigueParameters(
      1.0, 2.0, 3.0, 4.0);
  internal_forces::muscles::FatigueParameters shallowCopy(fatigueParameters);
  internal_forces::muscles::FatigueParameters deepCopyNow(
      fatigueParameters.DeepCopy());
  internal_forces::muscles::FatigueParameters deepCopyLater;
  deepCopyLater.DeepCopy(fatigueParameters);

  {
    SCALAR_TO_DOUBLE(recoveryRate, fatigueParameters.recoveryRate());
    SCALAR_TO_DOUBLE(shallowCopyRecoveryRate, shallowCopy.recoveryRate());
    SCALAR_TO_DOUBLE(deepCopyNowRecoveryRate, deepCopyNow.recoveryRate());
    SCALAR_TO_DOUBLE(deepCopyLaterRecoveryRate, deepCopyLater.recoveryRate());
    EXPECT_NEAR(recoveryRate, 2., requiredPrecision);
    EXPECT_NEAR(shallowCopyRecoveryRate, 2., requiredPrecision);
    EXPECT_NEAR(deepCopyNowRecoveryRate, 2., requiredPrecision);
    EXPECT_NEAR(deepCopyLaterRecoveryRate, 2., requiredPrecision);
  }

  fatigueParameters.setRecoveryRate(5.0);

  {
    SCALAR_TO_DOUBLE(recoveryRate, fatigueParameters.recoveryRate());
    SCALAR_TO_DOUBLE(shallowCopyRecoveryRate, shallowCopy.recoveryRate());
    SCALAR_TO_DOUBLE(deepCopyNowRecoveryRate, deepCopyNow.recoveryRate());
    SCALAR_TO_DOUBLE(deepCopyLaterRecoveryRate, deepCopyLater.recoveryRate());
    EXPECT_NEAR(recoveryRate, 5., requiredPrecision);
    EXPECT_NEAR(shallowCopyRecoveryRate, 5., requiredPrecision);
    EXPECT_NEAR(deepCopyNowRecoveryRate, 2., requiredPrecision);
    EXPECT_NEAR(deepCopyLaterRecoveryRate, 2., requiredPrecision);
  }
}

TEST(MuscleGroup, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    muscleGroup.setName("newName");
    EXPECT_STREQ(muscleGroup.name().c_str(), "newName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    muscleGroup.setOrigin("newOriginName");
    EXPECT_STREQ(muscleGroup.origin().c_str(), "newOriginName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    muscleGroup.setInsertion("newInsertionName");
    EXPECT_STREQ(muscleGroup.insertion().c_str(), "newInsertionName");
  }

  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    muscleGroup.addMuscle(
        "newMuscleName",
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    // Check the id of the last muscle added
    EXPECT_EQ(muscleGroup.nbMuscles(), 4);
    int idNewMuscle(muscleGroup.muscleID("newMuscleName"));
    EXPECT_EQ(idNewMuscle, 3);

    // Fetch new muscle from muscle
    EXPECT_STREQ(muscleGroup.muscle(3).name().c_str(), "newMuscleName");
  }
}

TEST(MuscleGroup, AddMuscle) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    internal_forces::muscles::IdealizedActuator muscleToAdd;
    muscleGroup.addMuscle(muscleToAdd);

    // Check number of muscle
    EXPECT_EQ(muscleGroup.nbMuscles(), 4);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    // Check number of muscle
    EXPECT_EQ(muscleGroup.nbMuscles(), 3);

    // Add muscle to muscle group
    muscleGroup.addMuscle(
        "newMuscleName",
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_TYPE::SIMPLE_STATE,
        internal_forces::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    // Check the number of muscles again
    EXPECT_EQ(muscleGroup.nbMuscles(), 4);

    // Add HILL muscle to muscle group
    muscleGroup.addMuscle(
        "newHillMuscle",
        internal_forces::muscles::MUSCLE_TYPE::HILL,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_TYPE::DYNAMIC,
        internal_forces::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    // Check the number of muscles again
    EXPECT_EQ(muscleGroup.nbMuscles(), 5);

    // Add HILL THELEN muscle to muscle group
    muscleGroup.addMuscle(
        "newHillThelenMuscle",
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_TYPE::BUCHANAN,
        internal_forces::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    // Check the number of muscles again
    EXPECT_EQ(muscleGroup.nbMuscles(), 6);

    // Add muscle to muscle group
    muscleGroup.addMuscle(
        "newHillThelenFatigable",
        internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_TYPE::SIMPLE_STATE,
        internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    // Check the number of muscles again
    EXPECT_EQ(muscleGroup.nbMuscles(), 7);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    // Check number of muscle
    EXPECT_EQ(muscleGroup.nbMuscles(), 3);

    // Add muscle to muscle group
    muscleGroup.addMuscle(
        "newMuscleName",
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        model.muscleGroup(0).muscle(0).pathModifier(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    // Check the number of muscles again
    EXPECT_EQ(muscleGroup.nbMuscles(), 4);
  }
}

TEST(MuscleGroup, DeepCopy) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    internal_forces::muscles::MuscleGroup shallowCopy(muscleGroup);
    internal_forces::muscles::MuscleGroup deepCopyNow(muscleGroup.DeepCopy());
    internal_forces::muscles::MuscleGroup deepCopyLater;
    deepCopyLater.DeepCopy(muscleGroup);

    EXPECT_STREQ(muscleGroup.name().c_str(), "base_to_r_ulna_radius_hand");
    EXPECT_STREQ(shallowCopy.name().c_str(), "base_to_r_ulna_radius_hand");
    EXPECT_STREQ(deepCopyNow.name().c_str(), "base_to_r_ulna_radius_hand");
    EXPECT_STREQ(deepCopyLater.name().c_str(), "base_to_r_ulna_radius_hand");

    // Set new name
    muscleGroup.setName("newMuscleGroupName");

    EXPECT_STREQ(muscleGroup.name().c_str(), "newMuscleGroupName");
    EXPECT_STREQ(shallowCopy.name().c_str(), "newMuscleGroupName");
    EXPECT_STREQ(deepCopyNow.name().c_str(), "base_to_r_ulna_radius_hand");
    EXPECT_STREQ(deepCopyLater.name().c_str(), "base_to_r_ulna_radius_hand");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

    muscleGroup.addMuscle(
        "newIdealizedActuator",
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    muscleGroup.addMuscle(
        "newHillActuator",
        internal_forces::muscles::MUSCLE_TYPE::HILL,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    internal_forces::muscles::MuscleGroup shallowCopy(muscleGroup);
    internal_forces::muscles::MuscleGroup deepCopyNow(muscleGroup.DeepCopy());
    internal_forces::muscles::MuscleGroup deepCopyLater;
    deepCopyLater.DeepCopy(muscleGroup);

    EXPECT_STREQ(muscleGroup.muscle(4).name().c_str(), "newHillActuator");
    EXPECT_STREQ(shallowCopy.muscle(4).name().c_str(), "newHillActuator");
    EXPECT_STREQ(deepCopyNow.muscle(4).name().c_str(), "newHillActuator");
    EXPECT_STREQ(deepCopyLater.muscle(4).name().c_str(), "newHillActuator");
  }
}

TEST(MuscleGroup, errors) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
    EXPECT_THROW(
        muscleGroup.addMuscle(
            "noTypeMuscle",
            internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE,
            model.muscleGroup(0).muscle(0).position(),
            model.muscleGroup(0).muscle(0).characteristics(),
            internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE),
        std::runtime_error);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
    EXPECT_THROW(muscleGroup.muscle(3), std::runtime_error);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
    EXPECT_THROW(
        muscleGroup.addMuscle(
            "noTypeMuscle",
            internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE,
            model.muscleGroup(0).muscle(0).position(),
            model.muscleGroup(0).muscle(0).characteristics(),
            internal_forces::muscles::STATE_TYPE::SIMPLE_STATE,
            internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE),
        std::runtime_error);
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
    EXPECT_THROW(
        muscleGroup.addMuscle(
            "noTypeMuscle",
            internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE,
            model.muscleGroup(0).muscle(0).position(),
            model.muscleGroup(0).muscle(0).characteristics(),
            model.muscleGroup(0).muscle(0).pathModifier(),
            internal_forces::muscles::STATE_TYPE::SIMPLE_STATE,
            internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE),
        std::runtime_error);
  }
}

TEST(Muscles, unitTest) {
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::Muscles muscles;
    muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");

    EXPECT_STREQ(muscles.muscleGroup(0).name().c_str(), "muscleGroupName");
    EXPECT_STREQ(muscles.muscleGroups()[0].name().c_str(), "muscleGroupName");
    EXPECT_STREQ(
        muscles.muscleGroup("muscleGroupName").origin().c_str(), "originName");
  }
  {
    Model model(modelPathForMuscleForce);
    internal_forces::muscles::Muscles muscles;
    muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");
    muscles.muscleGroup(0).addMuscle(
        "newIdealizedActuator",
        internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
        model.muscleGroup(0).muscle(0).position(),
        model.muscleGroup(0).muscle(0).characteristics(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    EXPECT_EQ(muscles.muscleNames().size(), 1);
  }
}

TEST(Muscles, errors) {
  Model model(modelPathForMuscleForce);
  internal_forces::muscles::Muscles muscles;
  muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");

  EXPECT_THROW(muscles.muscleGroup(1), std::runtime_error);
  EXPECT_THROW(muscles.muscleGroup("nameNoExists"), std::runtime_error);
}

TEST(Muscles, deepCopy) {
  Model model(modelPathForMuscleForce);
  internal_forces::muscles::Muscles muscles;
  muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");

  internal_forces::muscles::Muscles shallowCopy(muscles);
  internal_forces::muscles::Muscles deepCopyNow(muscles.DeepCopy());
  internal_forces::muscles::Muscles deepCopyLater;
  deepCopyLater.DeepCopy(muscles);

  EXPECT_STREQ(muscles.muscleGroup(0).name().c_str(), "muscleGroupName");
  EXPECT_STREQ(shallowCopy.muscleGroup(0).name().c_str(), "muscleGroupName");
  EXPECT_STREQ(deepCopyNow.muscleGroup(0).name().c_str(), "muscleGroupName");
  EXPECT_STREQ(deepCopyLater.muscleGroup(0).name().c_str(), "muscleGroupName");

  // Set new name
  muscles.muscleGroup(0).setName("newMuscleGroupName");

  // Check if copies changed names
  EXPECT_STREQ(muscles.muscleGroup(0).name().c_str(), "newMuscleGroupName");
  EXPECT_STREQ(shallowCopy.muscleGroup(0).name().c_str(), "newMuscleGroupName");
  EXPECT_STREQ(deepCopyNow.muscleGroup(0).name().c_str(), "newMuscleGroupName");
  EXPECT_STREQ(
      deepCopyLater.muscleGroup(0).name().c_str(), "newMuscleGroupName");
}

TEST(WrappingHalfCylinder, unitTest) {
  {
    utils::RotoTrans rt(
        utils::Vector3d(1., 1., 1.), utils::Vector3d(1., 1., 1.), "xyz");

    internal_forces::WrappingHalfCylinder wrappingHalfCylinder;

    wrappingHalfCylinder.setRadius(0.75);
    SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
    SCALAR_TO_DOUBLE(radius, wrappingHalfCylinder.radius());
    EXPECT_NEAR(diameter, 1.5, requiredPrecision);
    EXPECT_NEAR(radius, 0.75, requiredPrecision);

    wrappingHalfCylinder.setName("wrappingHalfCylinderName");
    EXPECT_STREQ(
        wrappingHalfCylinder.utils::Node::name().c_str(),
        "wrappingHalfCylinderName");
  }
  {
    utils::RotoTrans rt(
        utils::Vector3d(1., 1., 1.), utils::Vector3d(1., 1., 1.), "xyz");

    internal_forces::WrappingHalfCylinder wrappingHalfCylinder(rt, 0.25, 1.);
    SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
    EXPECT_NEAR(diameter, 0.5, requiredPrecision);
    SCALAR_TO_DOUBLE(length, wrappingHalfCylinder.length());
    EXPECT_NEAR(length, 1., requiredPrecision);
  }
  {
    utils::RotoTrans rt(
        utils::Vector3d(1., 1., 1.), utils::Vector3d(1., 1., 1.), "xyz");

    internal_forces::WrappingHalfCylinder wrappingHalfCylinder(
        rt, 0.5, 1., "name", "parentName");
    EXPECT_STREQ(wrappingHalfCylinder.parent().c_str(), "parentName");
  }
  {
    internal_forces::WrappingHalfCylinder wrappingHalfCylinder;
    utils::RotoTrans rt(
        utils::Vector3d(1., 1., 1.), utils::Vector3d(1., 1., 1.), "xyz");
    utils::Vector3d p1(1., 1., 1.);
    utils::Vector3d p2(2., 2., 2.);

    wrappingHalfCylinder.wrapPoints(
        rt, utils::Vector3d(0.5, 1., 1.5), utils::Vector3d(4., 5., 6.), p1, p2);

    SCALAR_TO_DOUBLE(p10, p1[0]);
    SCALAR_TO_DOUBLE(p11, p1[1]);
    SCALAR_TO_DOUBLE(p12, p1[2]);
    SCALAR_TO_DOUBLE(p20, p2[0]);
    SCALAR_TO_DOUBLE(p21, p2[1]);
    SCALAR_TO_DOUBLE(p22, p2[2]);
#ifdef BIORBD_USE_CASADI_MATH
    EXPECT_NEAR(p10, 0.71229470845913756, requiredPrecision);
    EXPECT_NEAR(p11, 1.1554478324299933, requiredPrecision);
    EXPECT_NEAR(p12, 0.90018809463370408, requiredPrecision);
    EXPECT_NEAR(p20, 0.71229470845913756, requiredPrecision);
    EXPECT_NEAR(p21, 1.1554478324299933, requiredPrecision);
    EXPECT_NEAR(p22, 0.90018809463370408, requiredPrecision);
#else
    EXPECT_NEAR(p10, 1.6666666666666665, requiredPrecision);
    EXPECT_NEAR(p11, 2.3333333333333335, requiredPrecision);
    EXPECT_NEAR(p12, 3.0, requiredPrecision);
    EXPECT_NEAR(p20, 2.8333333333333335, requiredPrecision);
    EXPECT_NEAR(p21, 3.666666666666667, requiredPrecision);
    EXPECT_NEAR(p22, 4.5, requiredPrecision);
#endif
  }
}

TEST(WrappingHalfCylinder, deepCopy) {
  Model model(modelPathForMuscleForce);
  utils::RotoTrans rt(
      utils::Vector3d(1, 1, 1), utils::Vector3d(1, 1, 1), "xyz");

  internal_forces::WrappingHalfCylinder wrappingHalfCylinder(rt, 0.25, 1.);

  internal_forces::WrappingHalfCylinder shallowCopy(wrappingHalfCylinder);
  internal_forces::WrappingHalfCylinder deepCopyNow(
      wrappingHalfCylinder.DeepCopy());
  internal_forces::WrappingHalfCylinder deepCopyLater;
  deepCopyLater.DeepCopy(wrappingHalfCylinder);

  {
    SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
    SCALAR_TO_DOUBLE(shallowCopyDiameter, shallowCopy.diameter());
    SCALAR_TO_DOUBLE(deepCopyNowDiameter, deepCopyNow.diameter());
    SCALAR_TO_DOUBLE(deepCopyLaterDiameter, deepCopyLater.diameter());
    EXPECT_NEAR(diameter, 0.5, requiredPrecision);
    EXPECT_NEAR(shallowCopyDiameter, 0.5, requiredPrecision);
    EXPECT_NEAR(deepCopyNowDiameter, 0.5, requiredPrecision);
    EXPECT_NEAR(deepCopyLaterDiameter, 0.5, requiredPrecision);

    SCALAR_TO_DOUBLE(length, wrappingHalfCylinder.length());
    SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.length());
    SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.length());
    SCALAR_TO_DOUBLE(deepCopyLaterlength, deepCopyLater.length());
    EXPECT_NEAR(length, 1.0, requiredPrecision);
    EXPECT_NEAR(shallowCopyLength, 1.0, requiredPrecision);
    EXPECT_NEAR(deepCopyNowLength, 1.0, requiredPrecision);
    EXPECT_NEAR(deepCopyLaterlength, 1.0, requiredPrecision);
  }

  // Set new diameter
  wrappingHalfCylinder.setRadius(1.0);
  wrappingHalfCylinder.setLength(2.5);

  // Check values
  {
    SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
    SCALAR_TO_DOUBLE(shallowCopyDiameter, shallowCopy.diameter());
    SCALAR_TO_DOUBLE(deepCopyNowDiameter, deepCopyNow.diameter());
    SCALAR_TO_DOUBLE(deepCopyLaterDiameter, deepCopyLater.diameter());
    EXPECT_NEAR(diameter, 2., requiredPrecision);
    EXPECT_NEAR(shallowCopyDiameter, 2., requiredPrecision);
    EXPECT_NEAR(deepCopyNowDiameter, 0.5, requiredPrecision);
    EXPECT_NEAR(deepCopyLaterDiameter, 0.5, requiredPrecision);

    SCALAR_TO_DOUBLE(length, wrappingHalfCylinder.length());
    SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.length());
    SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.length());
    SCALAR_TO_DOUBLE(deepCopyLaterlength, deepCopyLater.length());
    EXPECT_NEAR(length, 2.5, requiredPrecision);
    EXPECT_NEAR(shallowCopyLength, 2.5, requiredPrecision);
    EXPECT_NEAR(deepCopyNowLength, 1.0, requiredPrecision);
    EXPECT_NEAR(deepCopyLaterlength, 1.0, requiredPrecision);
  }
}

TEST(MuscleForce, position) {
  // TODO
  // Position of origin, insertion, and path in local
  // Position of origin, insertion, and path in global
}

TEST(MuscleForce, force) {
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  Q = Q.setOnes() / 10;
  Qdot = Qdot.setOnes() / 10;
  std::vector<std::shared_ptr<internal_forces::muscles::State>> states;
  for (size_t i = 0; i < model.nbMuscleTotal(); ++i) {
    states.push_back(
        std::make_shared<internal_forces::muscles::StateDynamics>(0, 0.2));
  }
  model.updateMuscles(Q, Qdot, true);

  const utils::Vector& F = model.muscleForces(states);

  std::vector<double> ExpectedForce(
      {165.19678913804927,
       178.49448510433558,
       90.789184279197713,
       92.59497473343656,
       74.287046497422935,
       198.53590160321016});
  for (unsigned int i = 0; i < model.nbMuscleTotal(); ++i) {
    SCALAR_TO_DOUBLE(val, F(i));
    EXPECT_NEAR(val, ExpectedForce[i], requiredPrecision);
  }
}

TEST(MuscleForce, torqueFromMuscles) {
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  Q.setOnes();
  Qdot.setOnes();
  std::vector<std::shared_ptr<internal_forces::muscles::State>> states;
  for (size_t i = 0; i < model.nbMuscleTotal(); ++i) {
    states.push_back(
        std::make_shared<internal_forces::muscles::StateDynamics>(0, 0.2));
  }

  rigidbody::GeneralizedTorque Tau(model);
  std::vector<double> TauExpected({-10.986834246856635, -1.7872663180818087});
  Tau = model.muscularJointTorque(states, Q, Qdot);
  for (unsigned int i = 0; i < Tau.size(); ++i) {
    SCALAR_TO_DOUBLE(val, Tau(i));
    EXPECT_NEAR(val, TauExpected[i], requiredPrecision);
  }

  rigidbody::GeneralizedAcceleration Qddot =
      model.ForwardDynamics(Q, Qdot, Tau);
  std::vector<double> QddotExpected({-47.412753236836856, 54.927407050792368});
  for (unsigned int i = 0; i < Qddot.size(); ++i) {
    SCALAR_TO_DOUBLE(val, Qddot(i));
    EXPECT_NEAR(val, QddotExpected[i], requiredPrecision);
  }
}

TEST(MuscleCharacterics, unittest) {
  {
    internal_forces::muscles::Characteristics charact;
    {
      SCALAR_TO_DOUBLE(optimalLength, charact.optimalLength());
      EXPECT_NEAR(optimalLength, 0, requiredPrecision);
    }
    double newOptimalLength(3.4);
    charact.setOptimalLength(newOptimalLength);
    {
      SCALAR_TO_DOUBLE(optimalLength, charact.optimalLength());
      EXPECT_NEAR(optimalLength, newOptimalLength, requiredPrecision);
    }
  }

  {
    internal_forces::muscles::Characteristics charact;
    {
      SCALAR_TO_DOUBLE(forceIsoMax, charact.forceIsoMax());
      EXPECT_NEAR(forceIsoMax, 0, requiredPrecision);
    }
    double newForceMax(156.9);
    charact.setForceIsoMax(newForceMax);

    {
      SCALAR_TO_DOUBLE(forceIsoMax, charact.forceIsoMax());
      EXPECT_NEAR(forceIsoMax, newForceMax, requiredPrecision);
    }
  }

  {
    internal_forces::muscles::Characteristics charact;
    {
      SCALAR_TO_DOUBLE(tendonSlackLength, charact.tendonSlackLength());
      EXPECT_NEAR(tendonSlackLength, 0, requiredPrecision);
    }
    double newTendonSlakLength(5.3);
    charact.setTendonSlackLength(newTendonSlakLength);
    {
      SCALAR_TO_DOUBLE(tendonSlackLength, charact.tendonSlackLength());
      EXPECT_NEAR(tendonSlackLength, newTendonSlakLength, requiredPrecision);
    }
  }

  {
    internal_forces::muscles::Characteristics charact;
    {
      SCALAR_TO_DOUBLE(pennationAngle, charact.pennationAngle());
      EXPECT_NEAR(pennationAngle, 0, requiredPrecision);
    }
    double newPennationAngle(1.09);
    charact.setPennationAngle(newPennationAngle);
    {
      SCALAR_TO_DOUBLE(pennationAngle, charact.pennationAngle());
      EXPECT_NEAR(pennationAngle, newPennationAngle, requiredPrecision);
    }
  }

  {
    internal_forces::muscles::Characteristics charact;
    {
      SCALAR_TO_DOUBLE(PCSA, charact.PCSA());
      EXPECT_NEAR(PCSA, 0, requiredPrecision);
    }
    double newPCSA(2.4);
    charact.setPCSA(newPCSA);
    {
      SCALAR_TO_DOUBLE(PCSA, charact.PCSA());
      EXPECT_NEAR(PCSA, newPCSA, requiredPrecision);
    }
  }
}

TEST(MuscleJacobian, jacobian) {
  Model model(modelPathForMuscleJacobian);
  rigidbody::GeneralizedCoordinates Q(model);
  Q = Q.setOnes() / 10;

  // Force computation of geometry
  internal_forces::muscles::Muscle& muscle(
      model.muscleGroup(muscleForMuscleJacobian)
          .muscle(muscleGroupForMuscleJacobian));
  EXPECT_THROW(muscle.position().jacobian(), std::runtime_error);
  model.updateMuscles(Q, true);

  size_t nRows(3 * (muscle.pathModifier().nbObjects() + 2));
  utils::Matrix jacoRef(nRows, model.nbQ());
  // Here we provide emperical values that we have confidence in
  jacoRef(0, 0) = 0.13689690274955996;
  jacoRef(0, 1) = 0;
  jacoRef(1, 0) = 0.0048155626519999824;
  jacoRef(1, 1) = 0;
  jacoRef(2, 0) = 0.0080659044260533528;
  jacoRef(2, 1) = 0;
  jacoRef(3, 0) = 0.1530001664829288;
  jacoRef(3, 1) = 0;
  jacoRef(4, 0) = -0.011356405676791911;
  jacoRef(4, 1) = 0;
  jacoRef(5, 0) = 0.0090532669706573556;
  jacoRef(5, 1) = 0;
  jacoRef(6, 0) = 0.22806179660848588;
  jacoRef(6, 1) = 0;
  jacoRef(7, 0) = -0.0097422466736260017;
  jacoRef(7, 1) = 0;
  jacoRef(8, 0) = 0.013478229213671932;
  jacoRef(8, 1) = 0;
  jacoRef(9, 0) = 0.2675513197235147;
  jacoRef(9, 1) = 0;
  jacoRef(10, 0) = 0.0086890497375387721;
  jacoRef(10, 1) = 0;
  jacoRef(11, 0) = 0.015765668988435677;
  jacoRef(11, 1) = 0;
  jacoRef(12, 0) = 0.28198829726211816;
  jacoRef(12, 1) = -0.0059281232164749703;
  jacoRef(13, 0) = 0.010784640478103213;
  jacoRef(13, 1) = -0.023460590547529164;
  jacoRef(14, 0) = 0.016612631083717935;
  jacoRef(14, 1) = 0.0013801799073302687;

  // Compare with computed values
  utils::Matrix jaco(muscle.position().jacobian());
  for (unsigned int i = 0; i < jaco.rows(); ++i) {
    for (unsigned int j = 0; j < jaco.cols(); ++j) {
      SCALAR_TO_DOUBLE(val, jaco(i, j));
      SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
      EXPECT_NEAR(val, valRef, requiredPrecision);
    }
  }

  // Change Q
  Q.setOnes();
  model.updateMuscles(Q, true);
  // Here we provide emperical values that we have confidence in
  jacoRef(0, 0) = 0.08134540996216065;
  jacoRef(0, 1) = 0;
  jacoRef(1, 0) = 0.11041411771499073;
  jacoRef(1, 1) = 0;
  jacoRef(2, 0) = 0.0045450332887922848;
  jacoRef(2, 1) = 0;
  jacoRef(3, 0) = 0.10400305103312649;
  jacoRef(3, 1) = 0;
  jacoRef(4, 0) = 0.1129992026935549;
  jacoRef(4, 1) = 0;
  jacoRef(5, 0) = 0.0058758916739148634;
  jacoRef(5, 1) = 0;
  jacoRef(6, 0) = 0.14940786497893349;
  jacoRef(6, 1) = 0;
  jacoRef(7, 0) = 0.17290229814238411;
  jacoRef(7, 1) = 0;
  jacoRef(8, 0) = 0.008416785894586807;
  jacoRef(8, 1) = 0;
  jacoRef(9, 0) = 0.1595464389865561;
  jacoRef(9, 1) = 0;
  jacoRef(10, 0) = 0.21534433421098692;
  jacoRef(10, 1) = 0;
  jacoRef(11, 0) = 0.0089171811535144387;
  jacoRef(11, 1) = 0;
  jacoRef(12, 0) = 0.17634886417858722;
  jacoRef(12, 1) = 0.024220863439657705;
  jacoRef(13, 0) = 0.24680272575831863;
  jacoRef(13, 1) = -0.00046752133823085851;
  jacoRef(14, 0) = 0.0098360540170798361;
  jacoRef(14, 1) = 0.00076029489073317982;

  // Compare with computed values
  jaco = muscle.position().jacobian();
  for (unsigned int i = 0; i < jaco.rows(); ++i) {
    for (unsigned int j = 0; j < jaco.cols(); ++j) {
      SCALAR_TO_DOUBLE(val, jaco(i, j));
      SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
      EXPECT_NEAR(val, valRef, requiredPrecision);
    }
  }
}

TEST(MuscleJacobian, jacobianLength) {
  Model model(modelPathForMuscleJacobian);
  rigidbody::GeneralizedCoordinates Q(model);
  Q = Q.setOnes() / 10;
  model.updateMuscles(Q, true);

  size_t nRows(model.nbMuscleTotal());
  utils::Matrix jacoRef(nRows, model.nbQ());
  // Here we provide emperical values that we have confidence in
  jacoRef(0, 0) = 0.037620360527045288;
  jacoRef(0, 1) = 0.022230374109936522;
  jacoRef(1, 0) = -0.017006708057341259;
  jacoRef(1, 1) = -0.012779004692822197;
  jacoRef(2, 0) = -0.0016529882883160136;
  jacoRef(2, 1) = -0.012779004692822197;
  jacoRef(3, 0) = 2.6526502766149064e-17;
  jacoRef(3, 1) = 0.022230374109936522;
  jacoRef(4, 0) = 5.2979083494670027e-18;
  jacoRef(4, 1) = 0.022230374109936522;
  jacoRef(5, 0) = 5.3888967663972615e-18;
  jacoRef(5, 1) = 0.00064415763125857495;

  // Compare with computed values
  utils::Matrix jaco(model.musclesLengthJacobian(Q));
  for (unsigned int i = 0; i < jaco.rows(); ++i) {
    for (unsigned int j = 0; j < jaco.cols(); ++j) {
      SCALAR_TO_DOUBLE(val, jaco(i, j));
      SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
      EXPECT_NEAR(val, valRef, requiredPrecision);
    }
  }

  // Change Q
  Q.setOnes();
  model.updateMuscles(Q, true);
  // Here we provide emperical values that we have confidence in
  jacoRef(0, 0) = 0.025994784772561841;
  jacoRef(0, 1) = 0.021586396946686921;
  jacoRef(1, 0) = -0.011088003983559729;
  jacoRef(1, 1) = -0.039462418895871744;
  jacoRef(2, 0) = 0.032285277521606391;
  jacoRef(2, 1) = -0.039462418895871744;
  jacoRef(3, 0) = 1.1900679209044559e-19;
  jacoRef(3, 1) = 0.021586396946686921;
  jacoRef(4, 0) = 1.4848744796707674e-17;
  jacoRef(4, 1) = 0.021586396946686921;
  jacoRef(5, 0) = -2.5088055592238924e-17;
  jacoRef(5, 1) = -0.015991501107813871;
  jaco = model.musclesLengthJacobian(Q);
  for (unsigned int i = 0; i < jaco.rows(); ++i) {
    for (unsigned int j = 0; j < jaco.cols(); ++j) {
      SCALAR_TO_DOUBLE(val, jaco(i, j));
      SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
      EXPECT_NEAR(val, valRef, requiredPrecision);
    }
  }
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaDerivativeViaPointers) {
  // Prepare the model
  Model model(modelPathForXiaDerivativeTest);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  Q.setZero();
  Qdot.setZero();
  model.updateMuscles(Q, Qdot, true);

  {
    internal_forces::muscles::HillDeGrooteTypeFatigable muscle(
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest));
    internal_forces::muscles::FatigueDynamicState& fatigueModel(
        dynamic_cast<internal_forces::muscles::FatigueDynamicState&>(
            muscle.fatigueState()));
    // Sanity check for the fatigue model
    EXPECT_EQ(
        fatigueModel.getType(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    // Initial value sanity check
    {
      SCALAR_TO_DOUBLE(activeFibersDot, fatigueModel.activeFibersDot());
      SCALAR_TO_DOUBLE(fatiguedFibersDot, fatigueModel.fatiguedFibersDot());
      SCALAR_TO_DOUBLE(restingFibersDot, fatigueModel.restingFibersDot());
      EXPECT_NEAR(activeFibersDot, 0, requiredPrecision);
      EXPECT_NEAR(fatiguedFibersDot, 0, requiredPrecision);
      EXPECT_NEAR(restingFibersDot, 0, requiredPrecision);
    }

    // Apply the derivative
    internal_forces::muscles::StateDynamics emg(
        0,
        activationEmgForXiaDerivativeTest);  // Set target
    fatigueModel.setState(
        currentActiveFibersForXiaDerivativeTest,
        currentFatiguedFibersForXiaDerivativeTest,
        currentRestingFibersForXiaDerivativeTest);
    fatigueModel.timeDerivativeState(
        emg,
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest)
            .characteristics());

    // Check the values
    EXPECT_NEAR(
        fatigueModel.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.fatiguedFibersDot(),
        expectedFatiguedDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.restingFibersDot(),
        expectedRestingDotForXiaDerivativeTest,
        requiredPrecision);
  }

  // Values should be changed in the model itself
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable muscle(
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest));
    internal_forces::muscles::FatigueDynamicState& fatigueModel(
        dynamic_cast<internal_forces::muscles::FatigueDynamicState&>(
            muscle.fatigueState()));

    // Check the values
    EXPECT_NEAR(
        fatigueModel.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.fatiguedFibersDot(),
        expectedFatiguedDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.restingFibersDot(),
        expectedRestingDotForXiaDerivativeTest,
        requiredPrecision);
  }
}
#endif

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaDerivativeViaInterface) {
  // Prepare the model
  Model model(modelPathForXiaDerivativeTest);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  Q.setZero();
  Qdot.setZero();
  model.updateMuscles(Q, Qdot, true);

  {
    internal_forces::muscles::HillDeGrooteTypeFatigable muscle(
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest));

    // Apply the derivative
    internal_forces::muscles::StateDynamics emg(
        0,
        activationEmgForXiaDerivativeTest);  // Set target
    muscle.setFatigueState(
        currentActiveFibersForXiaDerivativeTest,
        currentFatiguedFibersForXiaDerivativeTest,
        currentRestingFibersForXiaDerivativeTest);
    muscle.computeTimeDerivativeState(emg);
  }

  // Values should be changed in the model itself
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable muscle(
        model.muscleGroup(0).muscle(0));
    internal_forces::muscles::FatigueDynamicState& fatigueModel(
        dynamic_cast<internal_forces::muscles::FatigueDynamicState&>(
            muscle.fatigueState()));

    // Check the values
    EXPECT_NEAR(
        fatigueModel.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.fatiguedFibersDot(),
        expectedFatiguedDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.restingFibersDot(),
        expectedRestingDotForXiaDerivativeTest,
        requiredPrecision);
  }
}
#endif

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaDerivativeShallowViaCopy) {
  // Prepare the model
  Model model(modelPathForXiaDerivativeTest);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  Q.setZero();
  Qdot.setZero();
  model.updateMuscles(Q, Qdot, true);

  {
    internal_forces::muscles::HillDeGrooteTypeFatigable muscle(
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest));
    internal_forces::muscles::FatigueDynamicStateXia fatigueModel(
        dynamic_cast<internal_forces::muscles::FatigueDynamicStateXia&>(
            muscle.fatigueState()));
    // Sanity check for the fatigue model
    EXPECT_EQ(
        fatigueModel.getType(),
        internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    // Initial value sanity check
    {
      SCALAR_TO_DOUBLE(activeFibersDot, fatigueModel.activeFibersDot());
      SCALAR_TO_DOUBLE(fatiguedFibersDot, fatigueModel.fatiguedFibersDot());
      SCALAR_TO_DOUBLE(restingFibersDot, fatigueModel.restingFibersDot());
      EXPECT_NEAR(activeFibersDot, 0, requiredPrecision);
      EXPECT_NEAR(fatiguedFibersDot, 0, requiredPrecision);
      EXPECT_NEAR(restingFibersDot, 0, requiredPrecision);
    }

    // Apply the derivative
    internal_forces::muscles::StateDynamics emg(
        0,
        activationEmgForXiaDerivativeTest);  // Set target
    fatigueModel.setState(
        currentActiveFibersForXiaDerivativeTest,
        currentFatiguedFibersForXiaDerivativeTest,
        currentRestingFibersForXiaDerivativeTest);
    fatigueModel.timeDerivativeState(
        emg,
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest)
            .characteristics());

    // Check the values
    EXPECT_NEAR(
        fatigueModel.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.fatiguedFibersDot(),
        expectedFatiguedDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.restingFibersDot(),
        expectedRestingDotForXiaDerivativeTest,
        requiredPrecision);
  }

  // Values should be changed in the model itself since everything is
  // shallowcopied
  {
    internal_forces::muscles::HillDeGrooteTypeFatigable muscle(
        model.muscleGroup(muscleGroupForXiaDerivativeTest)
            .muscle(muscleForXiaDerivativeTest));
    internal_forces::muscles::FatigueDynamicState& fatigueModel(
        dynamic_cast<internal_forces::muscles::FatigueDynamicState&>(
            muscle.fatigueState()));

    // Check the values
    EXPECT_NEAR(
        fatigueModel.activeFibersDot(),
        expectedActivationDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.fatiguedFibersDot(),
        expectedFatiguedDotForXiaDerivativeTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.restingFibersDot(),
        expectedRestingDotForXiaDerivativeTest,
        requiredPrecision);
  }
}
#endif

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaSetStateLimitsTest) {
  // Prepare the model
  Model model(modelPathForXiaDerivativeTest);
  internal_forces::muscles::HillDeGrooteTypeFatigable muscle(
      model.muscleGroup(muscleGroupForXiaDerivativeTest)
          .muscle(muscleForXiaDerivativeTest));
  internal_forces::muscles::FatigueDynamicStateXia fatigueModel(
      dynamic_cast<internal_forces::muscles::FatigueDynamicStateXia&>(
          muscle.fatigueState()));
  // Sanity check for the fatigue model
  EXPECT_EQ(
      fatigueModel.getType(),
      internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

  // Teast limit active quantity
  {
    // Set values under 0
    fatigueModel.setState(
        negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest);

    // Check the values
    EXPECT_NEAR(fatigueModel.activeFibers(), 0, requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.fatiguedFibers(),
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.restingFibers(),
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest +
            negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
        requiredPrecision);

    // Set values over 1
    fatigueModel.setState(
        excessiveFibersQuantityForFatigueXiaSetStateLimitsTest,
        0,
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest -
            excessiveFibersQuantityForFatigueXiaSetStateLimitsTest);

    // Check the values
    EXPECT_NEAR(
        fatigueModel.activeFibers(),
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
        requiredPrecision);
    EXPECT_NEAR(fatigueModel.fatiguedFibers(), 0, requiredPrecision);
    EXPECT_NEAR(fatigueModel.restingFibers(), 0, requiredPrecision);
  }

  // Test limit resting quantity
  {
    // Set values to be corrected
    fatigueModel.setState(
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
        negativeFibersQuantityForFatigueXiaSetStateLimitsTest);

    // Check the values
    EXPECT_NEAR(
        fatigueModel.activeFibers(),
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest +
            negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
        requiredPrecision);
    EXPECT_NEAR(
        fatigueModel.fatiguedFibers(),
        positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
        requiredPrecision);
    EXPECT_NEAR(fatigueModel.restingFibers(), 0, requiredPrecision);

    // Set incorrect values
    EXPECT_THROW(
        fatigueModel.setState(
            positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
            positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
            excessiveFibersQuantityForFatigueXiaSetStateLimitsTest),
        std::runtime_error);
  }

  // Test limit fatigued quantity
  {
    // Set values
    EXPECT_THROW(
        fatigueModel.setState(
            positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
            negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
            positiveFibersQuantityForFatigueXiaSetStateLimitsTest),
        std::runtime_error);
    EXPECT_THROW(
        fatigueModel.setState(
            positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
            excessiveFibersQuantityForFatigueXiaSetStateLimitsTest,
            positiveFibersQuantityForFatigueXiaSetStateLimitsTest),
        std::runtime_error);
  }
}
#endif

#ifdef MODULE_STATIC_OPTIM

TEST(StaticOptim, OneFrameNoActivations) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  // Proceed with the static optimization
  auto optim = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, 0.01, 2, true, 0, 100);
  optim.run();
  auto muscleActivations = optim.finalSolution()[0];

  std::vector<double> expectedActivations = {
      9.999216994e-05,
      0.09403925441,
      9.9993900426726198e-05,
      0.04161937275,
      0.03119033903,
      0.0001000112881};
  for (size_t i = 0; i < expectedActivations.size(); ++i) {
    EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
  }

  auto muscleResiduals = optim.finalResidual()[0];
  std::vector<double> expectedResiduals = {1.684193518e-05, -2.996892531e-06};
  for (size_t i = 0; i < expectedResiduals.size(); ++i) {
    EXPECT_NEAR(muscleResiduals[i], expectedResiduals[i], 1e-6);
    EXPECT_NEAR(optim.finalResidual(0)[i], expectedResiduals[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, OneFrameOneActivationDouble) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  // Proceed with the static optimization
  double initialActivationGuess = 0.5;
  auto optim = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, initialActivationGuess, 2, true, 0, 100);
  optim.run();
  auto muscleActivations = optim.finalSolution()[0];

  std::vector<double> expectedActivations = {
      9.999226004e-05,
      0.09223102777,
      0.0002251981941,
      0.03982160002,
      0.03170127772,
      0.0001000122471};
  for (size_t i = 0; i < expectedActivations.size(); ++i) {
    EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
  }

  auto muscleResiduals = optim.finalResidual()[0];
  std::vector<double> expectedResiduals = {1.616486747e-05, -2.86732263e-06};
  for (size_t i = 0; i < expectedResiduals.size(); ++i) {
    EXPECT_NEAR(muscleResiduals[i], expectedResiduals[i], 1e-6);
    EXPECT_NEAR(optim.finalResidual(0)[i], expectedResiduals[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, OneFrameOneActivationVector) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  // Proceed with the static optimization
  utils::Vector initialActivationGuess(model.nbMuscles());
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    initialActivationGuess[i] = 0.5;
  }
  auto optim = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, initialActivationGuess, 2, true, 0, 100);
  optim.run();
  auto muscleActivations = optim.finalSolution()[0];

  std::vector<double> expectedActivations = {
      9.999226004e-05,
      0.09223102777,
      0.0002251981941,
      0.03982160002,
      0.03170127772,
      0.0001000122471};
  for (size_t i = 0; i < expectedActivations.size(); ++i) {
    EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
  }

  auto muscleResiduals = optim.finalResidual()[0];
  std::vector<double> expectedResiduals = {1.616486747e-05, -2.86732263e-06};
  for (size_t i = 0; i < expectedResiduals.size(); ++i) {
    EXPECT_NEAR(muscleResiduals[i], expectedResiduals[i], 1e-6);
    EXPECT_NEAR(optim.finalResidual(0)[i], expectedResiduals[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, MultiFrameNoActivation) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }
  std::vector<rigidbody::GeneralizedCoordinates> allQ;
  std::vector<rigidbody::GeneralizedVelocity> allQdot;
  std::vector<rigidbody::GeneralizedTorque> allTau;
  allQ.push_back(Q);
  allQ.push_back(Q);
  allQ.push_back(Q);
  allQdot.push_back(Qdot);
  allQdot.push_back(Qdot);
  allQdot.push_back(Qdot);
  allTau.push_back(Tau);
  allTau.push_back(Tau);
  allTau.push_back(Tau);

  // Proceed with the static optimization
  auto optim = internal_forces::muscles::StaticOptimization(
      model, allQ, allQdot, allTau, 0.01, 2, true, 0, 100);
  optim.run();
  auto allMuscleActivations = optim.finalSolution();
  EXPECT_EQ(allMuscleActivations.size(), allQ.size());

  // Each frame is linearized around the solution of the previous frame
  std::vector<std::vector<double>> expectedActivations = {
      {9.999216994e-05,
       0.09403925441,
       9.9993900426726198e-05,
       0.04161937275,
       0.03119033903,
       0.0001000112881},
      {9.999218296e-05,
       0.09364571407,
       0.0001051006059,
       0.04153329716,
       0.03120823323,
       0.000100011332},
      {9.999218291e-05,
       0.09364749152,
       0.0001051250158,
       0.04153286877,
       0.03120875352,
       0.000100011332}};
  for (size_t f = 0; f < allMuscleActivations.size(); ++f) {
    for (size_t i = 0; i < expectedActivations[f].size(); ++i) {
      EXPECT_NEAR(allMuscleActivations[f](i), expectedActivations[f][i], 1e-5);
    }
  }

  auto muscleResiduals = optim.finalResidual()[0];
  std::vector<double> expectedResiduals = {1.684193518e-05, -2.996892531e-06};
  for (size_t i = 0; i < expectedResiduals.size(); ++i) {
    EXPECT_NEAR(muscleResiduals[i], expectedResiduals[i], 1e-6);
    EXPECT_NEAR(optim.finalResidual(0)[i], expectedResiduals[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, MultiFrameActivationDouble) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }
  std::vector<rigidbody::GeneralizedCoordinates> allQ;
  std::vector<rigidbody::GeneralizedVelocity> allQdot;
  std::vector<rigidbody::GeneralizedTorque> allTau;
  allQ.push_back(Q);
  allQ.push_back(Q);
  allQ.push_back(Q);
  allQdot.push_back(Qdot);
  allQdot.push_back(Qdot);
  allQdot.push_back(Qdot);
  allTau.push_back(Tau);
  allTau.push_back(Tau);
  allTau.push_back(Tau);

  // Proceed with the static optimization
  double initialActivationGuess = 0.5;
  auto optim = internal_forces::muscles::StaticOptimization(
      model, allQ, allQdot, allTau, initialActivationGuess, 2, true, 0, 100);
  optim.run();
  auto allMuscleActivations = optim.finalSolution();
  EXPECT_EQ(allMuscleActivations.size(), allQ.size());

  // Each frame is linearized around the solution of the previous frame
  std::vector<std::vector<double>> expectedActivations = {
      {9.999226004e-05,
       0.09223102777,
       0.0002251981941,
       0.03982160002,
       0.03170127772,
       0.0001000122471},
      {9.999218277e-05,
       0.09365386563,
       0.0001052932453,
       0.04153165398,
       0.03120859221,
       0.0001000113332},
      {9.999218292e-05,
       0.09364743959,
       0.0001051784733,
       0.04153324528,
       0.03120829626,
       0.0001000113323}};
  for (size_t f = 0; f < allMuscleActivations.size(); ++f) {
    for (size_t i = 0; i < expectedActivations[f].size(); ++i) {
      EXPECT_NEAR(allMuscleActivations[f](i), expectedActivations[f][i], 1e-5);
    }
  }

  auto muscleResiduals = optim.finalResidual()[0];
  std::vector<double> expectedResiduals = {1.616486747e-05, -2.86732263e-06};
  for (size_t i = 0; i < expectedResiduals.size(); ++i) {
    EXPECT_NEAR(muscleResiduals[i], expectedResiduals[i], 1e-6);
    EXPECT_NEAR(optim.finalResidual(0)[i], expectedResiduals[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, MultiFrameNoActivationVector) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }
  std::vector<rigidbody::GeneralizedCoordinates> allQ;
  std::vector<rigidbody::GeneralizedVelocity> allQdot;
  std::vector<rigidbody::GeneralizedTorque> allTau;
  allQ.push_back(Q);
  allQ.push_back(Q);
  allQ.push_back(Q);
  allQdot.push_back(Qdot);
  allQdot.push_back(Qdot);
  allQdot.push_back(Qdot);
  allTau.push_back(Tau);
  allTau.push_back(Tau);
  allTau.push_back(Tau);

  // Proceed with the static optimization
  utils::Vector initialActivationGuess(model.nbMuscles());
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    initialActivationGuess[i] = 0.5;
  }
  auto optim = internal_forces::muscles::StaticOptimization(
      model, allQ, allQdot, allTau, initialActivationGuess, 2, true, 0, 100);
  optim.run();
  auto allMuscleActivations = optim.finalSolution();
  EXPECT_EQ(allMuscleActivations.size(), allQ.size());

  // Each frame is linearized around the solution of the previous frame
  std::vector<std::vector<double>> expectedActivations = {
      {9.999226004e-05,
       0.09223102777,
       0.0002251981941,
       0.03982160002,
       0.03170127772,
       0.0001000122471},
      {9.999218277e-05,
       0.09365386563,
       0.0001052932453,
       0.04153165398,
       0.03120859221,
       0.0001000113332},
      {9.999218292e-05,
       0.09364743959,
       0.0001051784733,
       0.04153324528,
       0.03120829626,
       0.0001000113323}};
  for (size_t f = 0; f < allMuscleActivations.size(); ++f) {
    for (size_t i = 0; i < expectedActivations[f].size(); ++i) {
      EXPECT_NEAR(allMuscleActivations[f](i), expectedActivations[f][i], 1e-5);
    }
  }

  auto muscleResiduals = optim.finalResidual()[0];
  std::vector<double> expectedResiduals = {1.616486747e-05, -2.86732263e-06};
  for (size_t i = 0; i < expectedResiduals.size(); ++i) {
    EXPECT_NEAR(muscleResiduals[i], expectedResiduals[i], 1e-6);
    EXPECT_NEAR(optim.finalResidual(0)[i], expectedResiduals[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, OneFrameNonLinearized) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  // Proceed with the static optimization
  auto optim =
      internal_forces::muscles::StaticOptimization(model, Q, Qdot, Tau);
  optim.run(false);
  auto muscleActivations = optim.finalSolution()[0];

  std::vector<double> expectedActivations = {
      9.999219019e-05,
      0.09364789063,
      0.000103664113,
      0.04147400652,
      0.03128558591,
      0.0001000113628};
  for (size_t i = 0; i < expectedActivations.size(); ++i) {
    EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
  }

  auto muscleResiduals = optim.finalResidual()[0];
  std::vector<double> expectedResiduals = {1.669578312e-05, -2.986397445e-06};
  for (size_t i = 0; i < expectedResiduals.size(); ++i) {
    EXPECT_NEAR(muscleResiduals[i], expectedResiduals[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, InvalidObjectiveScaling) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  Q.setZero();
  Qdot.setZero();
  Tau.setZero();

  for (double scaling : {0.0, -1.0}) {
    EXPECT_THROW(
        internal_forces::muscles::StaticOptimization(
            model, Q, Qdot, Tau, 0.01, 2, true, 0, scaling),
        std::runtime_error);
  }

#endif
}

TEST(StaticOptim, ObjectiveScaling) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  // Scaling up the objective makes IPOPT converge more tightly, especially for
  // the muscles at their lower bound (the third muscle is about 2e-5 away from
  // these values without the scaling)
  auto optim = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, 0.01, 2, true, 0, 1000);
  optim.run();
  auto muscleActivations = optim.finalSolution()[0];

  std::vector<double> expectedActivations = {
      9.999000217e-05,
      0.09404543467,
      0.0001000659485,
      0.04161043363,
      0.03118362845,
      9.999002129e-05};
  for (size_t i = 0; i < expectedActivations.size(); ++i) {
    EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-6);
  }

#endif
}

TEST(StaticOptim, CheckDerivatives) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  // Run the derivative checker of Ipopt on both formulations. As the values
  // depend on the convergence path, only the satisfaction of the constraints
  // is tested
  for (bool useLinearizedState : {true, false}) {
    auto optim = internal_forces::muscles::StaticOptimization(
        model, Q, Qdot, Tau, 0.01, 2, true, 0, 100);
    optim.run(useLinearizedState, 1e-7, 1000, true);
    auto muscleActivations = optim.finalSolution(0);
    auto muscleResiduals = optim.finalResidual(0);

    std::vector<std::shared_ptr<internal_forces::muscles::State>> states;
    for (size_t i = 0; i < model.nbMuscles(); ++i) {
      EXPECT_GE(muscleActivations[i], 0.0001 - 1e-8);
      EXPECT_LE(muscleActivations[i], 0.9999 + 1e-8);
      states.push_back(
          std::make_shared<internal_forces::muscles::State>(
              0, muscleActivations[i]));
    }
    if (!useLinearizedState) {
      rigidbody::GeneralizedTorque muscleTorque(
          model.muscularJointTorque(states, Q, Qdot));
      for (size_t i = 0; i < Tau.size(); ++i) {
        EXPECT_NEAR(muscleTorque[i] + muscleResiduals[i], Tau[i], 1e-6);
      }
    }
  }

#endif
}

TEST(StaticOptim, MaxIterationAndTolerance) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  // Not reaching convergence should only warn and still give a solution
  auto optim = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, 0.01, 2, true, 0, 100);
  EXPECT_NO_THROW(optim.run(true, 1e-12, 1));
  auto muscleActivations = optim.finalSolution(0);
  EXPECT_EQ(muscleActivations.size(), model.nbMuscles());
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_GE(muscleActivations[i], 0.0001 - 1e-8);
    EXPECT_LE(muscleActivations[i], 0.9999 + 1e-8);
  }

#endif
}

TEST(StaticOptim, StateDynamicsInitialGuess) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }
  std::vector<rigidbody::GeneralizedCoordinates> allQ = {Q, Q};
  std::vector<rigidbody::GeneralizedVelocity> allQdot = {Qdot, Qdot};
  std::vector<rigidbody::GeneralizedTorque> allTau = {Tau, Tau};

  utils::Vector activations(model.nbMuscles());
  std::vector<internal_forces::muscles::StateDynamics> states;
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    activations[i] = 0.1 + 0.1 * static_cast<double>(i);
    states.push_back(
        internal_forces::muscles::StateDynamics(0, activations[i]));
  }

  // Giving the initial guess as states or as a vector is the same problem
  auto optimStates = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, states, 2, true, 0, 100);
  auto optimVector = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, activations, 2, true, 0, 100);
  auto optimAllStates = internal_forces::muscles::StaticOptimization(
      model, allQ, allQdot, allTau, states, 2, true, 0, 100);
  auto optimAllVector = internal_forces::muscles::StaticOptimization(
      model, allQ, allQdot, allTau, activations, 2, true, 0, 100);
  optimStates.run();
  optimVector.run();
  optimAllStates.run();
  optimAllVector.run();
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_NEAR(
        optimStates.finalSolution(0)[i],
        optimVector.finalSolution(0)[i],
        1e-10);
    for (unsigned int f = 0; f < allQ.size(); ++f) {
      EXPECT_NEAR(
          optimAllStates.finalSolution(f)[i],
          optimAllVector.finalSolution(f)[i],
          1e-10);
    }
  }

#endif
}

TEST(StaticOptim, RunTwice) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }
  std::vector<rigidbody::GeneralizedCoordinates> allQ = {Q, Q};
  std::vector<rigidbody::GeneralizedVelocity> allQdot = {Qdot, Qdot};
  std::vector<rigidbody::GeneralizedTorque> allTau = {Tau, Tau};

  auto optim = internal_forces::muscles::StaticOptimization(
      model, allQ, allQdot, allTau, 0.01, 2, true, 0, 100);
  optim.run(true);
  auto firstSolution = optim.finalSolution();
  auto firstResidual = optim.finalResidual();

  // Running again restarts from the initial guess, so the results are the
  // same and the previous problems are discarded
  optim.run(true);
  auto secondSolution = optim.finalSolution();
  auto secondResidual = optim.finalResidual();
  ASSERT_EQ(secondSolution.size(), allQ.size());
  for (size_t f = 0; f < allQ.size(); ++f) {
    for (size_t i = 0; i < model.nbMuscles(); ++i) {
      EXPECT_NEAR(secondSolution[f][i], firstSolution[f][i], 1e-10);
    }
    for (size_t i = 0; i < Tau.size(); ++i) {
      EXPECT_NEAR(secondResidual[f][i], firstResidual[f][i], 1e-10);
    }
  }

  // Running with the other formulation replaces the results
  optim.run(false);
  EXPECT_EQ(optim.finalSolution().size(), allQ.size());

#endif
}

TEST(StaticOptim, PNormFactor) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);

  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }

  auto optim = internal_forces::muscles::StaticOptimization(
      model, Q, Qdot, Tau, 0.01, 3, true, 0, 100);
  optim.run(false);
  auto muscleActivations = optim.finalSolution(0);
  auto muscleResiduals = optim.finalResidual(0);

  std::vector<std::shared_ptr<internal_forces::muscles::State>> states;
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_GE(muscleActivations[i], 0.0001 - 1e-8);
    EXPECT_LE(muscleActivations[i], 0.9999 + 1e-8);
    states.push_back(
        std::make_shared<internal_forces::muscles::State>(
            0, muscleActivations[i]));
  }
  rigidbody::GeneralizedTorque muscleTorque(
      model.muscularJointTorque(states, Q, Qdot));
  for (size_t i = 0; i < Tau.size(); ++i) {
    EXPECT_NEAR(muscleTorque[i] + muscleResiduals[i], Tau[i], 1e-6);
  }

#endif
}

#ifndef BIORBD_USE_CASADI_MATH
static void fillStaticOptimStates(
    Model& model,
    rigidbody::GeneralizedCoordinates& Q,
    rigidbody::GeneralizedVelocity& Qdot,
    rigidbody::GeneralizedTorque& Tau,
    utils::Vector& activations) {
  for (size_t i = 0; i < Q.size(); ++i) {
    Q[i] = static_cast<double>(i) * 1.1;
    Qdot[i] = static_cast<double>(i) * 1.1;
    Tau[i] = static_cast<double>(i) * 1.1;
  }
  activations = utils::Vector(model.nbMuscles());
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    activations[i] = 0.1 + 0.1 * static_cast<double>(i);
  }
}

static std::vector<double> staticOptimTestPoint(
    const utils::Vector& activations,
    const std::vector<double>& residuals) {
  std::vector<double> x;
  for (size_t i = 0; i < static_cast<size_t>(activations.size()); ++i) {
    x.push_back(activations[i]);
  }
  x.insert(x.end(), residuals.begin(), residuals.end());
  return x;
}

static rigidbody::GeneralizedTorque staticOptimMuscleTorque(
    Model& model,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const std::vector<double>& x) {
  std::vector<std::shared_ptr<internal_forces::muscles::State>> states;
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    states.push_back(
        std::make_shared<internal_forces::muscles::State>(0, x[i]));
  }
  return model.muscularJointTorque(states, Q, Qdot);
}

// Compare the jacobian of the constraints given by the problem to central
// finite differences of the constraints
static void checkStaticOptimJacobian(
    internal_forces::muscles::StaticOptimizationIpopt& problem,
    const std::vector<double>& x,
    double tolerance) {
  Ipopt::Index n, m, nnzJac, nnzHess;
  Ipopt::TNLP::IndexStyleEnum indexStyle;
  problem.get_nlp_info(n, m, nnzJac, nnzHess, indexStyle);

  std::vector<Ipopt::Index> iRow(nnzJac), jCol(nnzJac);
  std::vector<double> values(nnzJac);
  problem.eval_jac_g(
      n, x.data(), true, m, nnzJac, iRow.data(), jCol.data(), nullptr);
  problem.eval_jac_g(
      n, x.data(), true, m, nnzJac, nullptr, nullptr, values.data());

  utils::Matrix jacobian(utils::Matrix::Zero(m, n));
  for (Ipopt::Index k = 0; k < nnzJac; ++k) {
    jacobian(iRow[k], jCol[k]) = values[k];
  }

  double h(1e-6);
  std::vector<double> gPlus(m), gMinus(m);
  for (Ipopt::Index j = 0; j < n; ++j) {
    std::vector<double> xPlus(x), xMinus(x);
    xPlus[j] += h;
    xMinus[j] -= h;
    problem.eval_g(n, xPlus.data(), true, m, gPlus.data());
    problem.eval_g(n, xMinus.data(), true, m, gMinus.data());
    for (Ipopt::Index i = 0; i < m; ++i) {
      double expected((gPlus[i] - gMinus[i]) / (2 * h));
      EXPECT_NEAR(
          jacobian(i, j),
          expected,
          tolerance * std::max(1.0, std::fabs(expected)));
    }
  }
}
#endif

TEST(StaticOptimIpopt, NlpInfo) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  Ipopt::Index nbMus(static_cast<Ipopt::Index>(model.nbMuscles()));
  Ipopt::Index nbTau(static_cast<Ipopt::Index>(model.nbGeneralizedTorque()));
  for (bool useResidual : {true, false}) {
    internal_forces::muscles::StaticOptimizationIpopt problem(
        model, Q, Qdot, Tau, activations, useResidual);
    Ipopt::Index n, m, nnzJac, nnzHess;
    Ipopt::TNLP::IndexStyleEnum indexStyle;
    EXPECT_TRUE(problem.get_nlp_info(n, m, nnzJac, nnzHess, indexStyle));
    EXPECT_EQ(indexStyle, Ipopt::TNLP::C_STYLE);
    EXPECT_EQ(m, nbTau);
    if (useResidual) {
      EXPECT_EQ(n, nbMus + static_cast<Ipopt::Index>(model.nbQ()));
      EXPECT_EQ(nnzJac, (nbMus + 1) * nbTau);
    } else {
      EXPECT_EQ(n, nbMus);
      EXPECT_EQ(nnzJac, nbMus * nbTau);
    }
  }

#endif
}

TEST(StaticOptimIpopt, BoundsInfo) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  internal_forces::muscles::StaticOptimizationIpopt problem(
      model, Q, Qdot, Tau, activations);
  Ipopt::Index n, m, nnzJac, nnzHess;
  Ipopt::TNLP::IndexStyleEnum indexStyle;
  problem.get_nlp_info(n, m, nnzJac, nnzHess, indexStyle);

  std::vector<double> xL(n), xU(n), gL(m), gU(m);
  EXPECT_TRUE(problem.get_bounds_info(
      n, xL.data(), xU.data(), m, gL.data(), gU.data()));
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_DOUBLE_EQ(xL[i], 0.0001);
    EXPECT_DOUBLE_EQ(xU[i], 0.9999);
  }
  for (size_t i = model.nbMuscles(); i < static_cast<size_t>(n); ++i) {
    EXPECT_DOUBLE_EQ(xL[i], -1000.0);
    EXPECT_DOUBLE_EQ(xU[i], 1000.0);
  }
  for (Ipopt::Index i = 0; i < m; ++i) {
    EXPECT_DOUBLE_EQ(gL[i], 0.0);
    EXPECT_DOUBLE_EQ(gU[i], 0.0);
  }

#endif
}

TEST(StaticOptimIpopt, StartingPoint) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  internal_forces::muscles::StaticOptimizationIpopt problem(
      model, Q, Qdot, Tau, activations);
  Ipopt::Index n, m, nnzJac, nnzHess;
  Ipopt::TNLP::IndexStyleEnum indexStyle;
  problem.get_nlp_info(n, m, nnzJac, nnzHess, indexStyle);

  std::vector<double> x(n, -1.0);
  EXPECT_TRUE(problem.get_starting_point(
      n, true, x.data(), false, nullptr, nullptr, m, false, nullptr));
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_DOUBLE_EQ(x[i], activations[i]);
  }
  for (size_t i = model.nbMuscles(); i < static_cast<size_t>(n); ++i) {
    EXPECT_DOUBLE_EQ(x[i], 0.0);
  }

#endif
}

TEST(StaticOptimIpopt, Objective) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  double activationNorm(0);
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    activationNorm += activations[i] * activations[i];
  }

  {
    // With residuals, they are weighted by 1000
    internal_forces::muscles::StaticOptimizationIpopt problem(
        model, Q, Qdot, Tau, activations);
    std::vector<double> x(staticOptimTestPoint(activations, {0.3, -0.7}));
    double obj;
    EXPECT_TRUE(problem.eval_f(
        static_cast<Ipopt::Index>(x.size()), x.data(), true, obj));
    EXPECT_NEAR(obj, activationNorm + 1000 * (0.3 * 0.3 + 0.7 * 0.7), 1e-10);

    // Without new_x, the previously dispatched values are used
    std::vector<double> xOther(x.size(), 0.5);
    EXPECT_TRUE(problem.eval_f(
        static_cast<Ipopt::Index>(x.size()), xOther.data(), false, obj));
    EXPECT_NEAR(obj, activationNorm + 1000 * (0.3 * 0.3 + 0.7 * 0.7), 1e-10);
  }
  {
    internal_forces::muscles::StaticOptimizationIpopt problem(
        model, Q, Qdot, Tau, activations, false);
    std::vector<double> x(staticOptimTestPoint(activations, {}));
    double obj;
    EXPECT_TRUE(problem.eval_f(
        static_cast<Ipopt::Index>(x.size()), x.data(), true, obj));
    EXPECT_NEAR(obj, activationNorm, 1e-10);
  }

#endif
}

TEST(StaticOptimIpopt, ObjectiveGradient) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  internal_forces::muscles::StaticOptimizationIpopt problem(
      model, Q, Qdot, Tau, activations);
  std::vector<double> x(staticOptimTestPoint(activations, {0.3, -0.7}));
  Ipopt::Index n(static_cast<Ipopt::Index>(x.size()));

  std::vector<double> grad(n);
  EXPECT_TRUE(problem.eval_grad_f(n, x.data(), true, grad.data()));
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_NEAR(grad[i], 2 * activations[i], 1e-10);
  }
  EXPECT_NEAR(grad[model.nbMuscles()], 2000 * 0.3, 1e-10);
  EXPECT_NEAR(grad[model.nbMuscles() + 1], -2000 * 0.7, 1e-10);

  // The objective is quadratic, so central finite differences are exact up to
  // the round-off errors
  double h(1e-6);
  for (Ipopt::Index j = 0; j < n; ++j) {
    std::vector<double> xPlus(x), xMinus(x);
    xPlus[j] += h;
    xMinus[j] -= h;
    double fPlus, fMinus;
    problem.eval_f(n, xPlus.data(), true, fPlus);
    problem.eval_f(n, xMinus.data(), true, fMinus);
    EXPECT_NEAR(grad[j], (fPlus - fMinus) / (2 * h), 1e-4);
  }

#endif
}

TEST(StaticOptimIpopt, ObjectivePNorm) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  internal_forces::muscles::StaticOptimizationIpopt problem(
      model, Q, Qdot, Tau, activations, true, 3);
  std::vector<double> x(staticOptimTestPoint(activations, {0.3, -0.7}));
  Ipopt::Index n(static_cast<Ipopt::Index>(x.size()));

  double activationNorm(0);
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    activationNorm += std::pow(activations[i], 3);
  }
  double obj;
  EXPECT_TRUE(problem.eval_f(n, x.data(), true, obj));
  EXPECT_NEAR(obj, activationNorm + 1000 * (0.3 * 0.3 + 0.7 * 0.7), 1e-10);

  std::vector<double> grad(n);
  EXPECT_TRUE(problem.eval_grad_f(n, x.data(), true, grad.data()));
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_NEAR(grad[i], 3 * activations[i] * activations[i], 1e-10);
  }
  double h(1e-6);
  for (Ipopt::Index j = 0; j < n; ++j) {
    std::vector<double> xPlus(x), xMinus(x);
    xPlus[j] += h;
    xMinus[j] -= h;
    double fPlus, fMinus;
    problem.eval_f(n, xPlus.data(), true, fPlus);
    problem.eval_f(n, xMinus.data(), true, fMinus);
    EXPECT_NEAR(grad[j], (fPlus - fMinus) / (2 * h), 1e-4);
  }

#endif
}

TEST(StaticOptimIpopt, FinalSolutionBeforeOptimization) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  // If Ipopt aborts before finalize_solution, the initial guess is returned
  internal_forces::muscles::StaticOptimizationIpopt problem(
      model, Q, Qdot, Tau, activations);
  utils::Vector solution(problem.finalSolution());
  utils::Vector residual(problem.finalResidual());
  EXPECT_EQ(solution.size(), model.nbMuscles());
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_DOUBLE_EQ(solution[i], activations[i]);
  }
  EXPECT_EQ(residual.size(), Tau.size());
  for (size_t i = 0; i < Tau.size(); ++i) {
    EXPECT_DOUBLE_EQ(residual[i], 0.0);
  }

#endif
}

TEST(StaticOptimIpopt, Constraints) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  internal_forces::muscles::StaticOptimizationIpopt problem(
      model, Q, Qdot, Tau, activations);
  std::vector<double> x(staticOptimTestPoint(activations, {0.3, -0.7}));
  Ipopt::Index n(static_cast<Ipopt::Index>(x.size()));
  Ipopt::Index m(static_cast<Ipopt::Index>(Tau.size()));

  std::vector<double> g(m);
  EXPECT_TRUE(problem.eval_g(n, x.data(), true, m, g.data()));
  rigidbody::GeneralizedTorque muscleTorque(
      staticOptimMuscleTorque(model, Q, Qdot, x));
  for (Ipopt::Index i = 0; i < m; ++i) {
    EXPECT_NEAR(
        g[i], muscleTorque[i] + x[model.nbMuscles() + i] - Tau[i], 1e-10);
  }

#endif
}

TEST(StaticOptimIpopt, ConstraintsJacobian) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  for (bool useResidual : {true, false}) {
    internal_forces::muscles::StaticOptimizationIpopt problem(
        model, Q, Qdot, Tau, activations, useResidual);
    Ipopt::Index n, m, nnzJac, nnzHess;
    Ipopt::TNLP::IndexStyleEnum indexStyle;
    problem.get_nlp_info(n, m, nnzJac, nnzHess, indexStyle);

    // The structure is dense for the muscles and diagonal for the residuals
    std::vector<Ipopt::Index> iRow(nnzJac), jCol(nnzJac);
    std::vector<double> x(n, 0.5);
    EXPECT_TRUE(problem.eval_jac_g(
        n, x.data(), true, m, nnzJac, iRow.data(), jCol.data(), nullptr));
    Ipopt::Index k(0);
    for (Ipopt::Index j = 0; j < static_cast<Ipopt::Index>(model.nbMuscles());
         ++j) {
      for (Ipopt::Index i = 0; i < m; ++i) {
        EXPECT_EQ(iRow[k], i);
        EXPECT_EQ(jCol[k++], j);
      }
    }
    for (Ipopt::Index i = 0; k < nnzJac; ++i) {
      EXPECT_EQ(iRow[k], i);
      EXPECT_EQ(jCol[k++], static_cast<Ipopt::Index>(model.nbMuscles()) + i);
    }

    std::vector<double> residuals;
    if (useResidual) {
      residuals = {0.3, -0.7};
    }
    checkStaticOptimJacobian(
        problem, staticOptimTestPoint(activations, residuals), 1e-5);
  }

#endif
}

TEST(StaticOptimIpopt, FinalizeSolution) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  internal_forces::muscles::StaticOptimizationIpopt problem(
      model, Q, Qdot, Tau, activations);
  std::vector<double> x(staticOptimTestPoint(activations, {0.3, -0.7}));
  problem.finalize_solution(
      Ipopt::SUCCESS,
      static_cast<Ipopt::Index>(x.size()),
      x.data(),
      nullptr,
      nullptr,
      static_cast<Ipopt::Index>(Tau.size()),
      nullptr,
      nullptr,
      0,
      nullptr,
      nullptr);

  utils::Vector solution(problem.finalSolution());
  utils::Vector residual(problem.finalResidual());
  EXPECT_EQ(solution.size(), model.nbMuscles());
  for (size_t i = 0; i < model.nbMuscles(); ++i) {
    EXPECT_DOUBLE_EQ(solution[i], activations[i]);
  }
  EXPECT_EQ(residual.size(), Tau.size());
  EXPECT_DOUBLE_EQ(residual[0], 0.3);
  EXPECT_DOUBLE_EQ(residual[1], -0.7);

#endif
}

TEST(StaticOptimIpopt, InvalidEpsilon) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  EXPECT_THROW(
      internal_forces::muscles::StaticOptimizationIpopt(
          model, Q, Qdot, Tau, activations, true, 2, 0, 1e-13),
      std::runtime_error);
  EXPECT_THROW(
      internal_forces::muscles::StaticOptimizationIpoptLinearized(
          model, Q, Qdot, Tau, activations, true, 2, 0, 1e-13),
      std::runtime_error);

#endif
}

TEST(StaticOptimIpoptLinearized, Constraints) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  internal_forces::muscles::StaticOptimizationIpoptLinearized problem(
      model, Q, Qdot, Tau, activations);
  Ipopt::Index m(static_cast<Ipopt::Index>(Tau.size()));
  std::vector<double> g(m);

  // At the reference activations (the initial guess) and at no activation,
  // the linearized torques are exact
  std::vector<double> residuals = {0.3, -0.7};
  utils::Vector noActivation(utils::Vector::Zero(model.nbMuscles()));
  for (const auto& reference : {activations, noActivation}) {
    std::vector<double> x(staticOptimTestPoint(reference, residuals));
    Ipopt::Index n(static_cast<Ipopt::Index>(x.size()));
    EXPECT_TRUE(problem.eval_g(n, x.data(), true, m, g.data()));
    rigidbody::GeneralizedTorque muscleTorque(
        staticOptimMuscleTorque(model, Q, Qdot, x));
    for (Ipopt::Index i = 0; i < m; ++i) {
      EXPECT_NEAR(g[i], muscleTorque[i] + residuals[i] - Tau[i], 1e-8);
    }
  }

  // The constraints are affine with respect to the activations
  std::vector<double> x0(staticOptimTestPoint(noActivation, {0, 0}));
  std::vector<double> x1(staticOptimTestPoint(activations, {0, 0}));
  std::vector<double> xHalf(x0.size());
  for (size_t i = 0; i < x0.size(); ++i) {
    xHalf[i] = 0.5 * (x0[i] + x1[i]);
  }
  Ipopt::Index n(static_cast<Ipopt::Index>(x0.size()));
  std::vector<double> g0(m), g1(m), gHalf(m);
  problem.eval_g(n, x0.data(), true, m, g0.data());
  problem.eval_g(n, x1.data(), true, m, g1.data());
  problem.eval_g(n, xHalf.data(), true, m, gHalf.data());
  for (Ipopt::Index i = 0; i < m; ++i) {
    EXPECT_NEAR(gHalf[i], 0.5 * (g0[i] + g1[i]), 1e-10);
  }

#endif
}

TEST(StaticOptimIpoptLinearized, ConstraintsJacobian) {
#ifdef BIORBD_USE_CASADI_MATH
  std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
  Model model(modelPathForMuscleForce);
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  rigidbody::GeneralizedTorque Tau(model);
  utils::Vector activations;
  fillStaticOptimStates(model, Q, Qdot, Tau, activations);

  for (bool useResidual : {true, false}) {
    internal_forces::muscles::StaticOptimizationIpoptLinearized problem(
        model, Q, Qdot, Tau, activations, useResidual);
    Ipopt::Index n, m, nnzJac, nnzHess;
    Ipopt::TNLP::IndexStyleEnum indexStyle;
    problem.get_nlp_info(n, m, nnzJac, nnzHess, indexStyle);

    // The structure is dense for the muscles and diagonal for the residuals
    std::vector<Ipopt::Index> iRow(nnzJac), jCol(nnzJac);
    std::vector<double> x(n, 0.5);
    EXPECT_TRUE(problem.eval_jac_g(
        n, x.data(), true, m, nnzJac, iRow.data(), jCol.data(), nullptr));
    Ipopt::Index k(0);
    for (Ipopt::Index j = 0; j < static_cast<Ipopt::Index>(model.nbMuscles());
         ++j) {
      for (Ipopt::Index i = 0; i < m; ++i) {
        EXPECT_EQ(iRow[k], i);
        EXPECT_EQ(jCol[k++], j);
      }
    }
    for (Ipopt::Index i = 0; k < nnzJac; ++i) {
      EXPECT_EQ(iRow[k], i);
      EXPECT_EQ(jCol[k++], static_cast<Ipopt::Index>(model.nbMuscles()) + i);
    }

    // The jacobian is the slope of each muscle between no activation and its
    // reference activation, whatever the point it is evaluated at
    std::vector<double> values(nnzJac), valuesOther(nnzJac);
    problem.eval_jac_g(
        n, x.data(), true, m, nnzJac, nullptr, nullptr, values.data());
    std::vector<double> xOther(n, 0.1);
    problem.eval_jac_g(
        n,
        xOther.data(),
        true,
        m,
        nnzJac,
        nullptr,
        nullptr,
        valuesOther.data());
    std::vector<double> zero(n, 0.0);
    rigidbody::GeneralizedTorque torqueZero(
        staticOptimMuscleTorque(model, Q, Qdot, zero));
    k = 0;
    for (size_t j = 0; j < model.nbMuscles(); ++j) {
      std::vector<double> xMuscle(n, 0.0);
      xMuscle[j] = activations[j];
      rigidbody::GeneralizedTorque torqueMuscle(
          staticOptimMuscleTorque(model, Q, Qdot, xMuscle));
      for (Ipopt::Index i = 0; i < m; ++i) {
        EXPECT_NEAR(
            values[k],
            (torqueMuscle[i] - torqueZero[i]) / activations[j],
            1e-8);
        EXPECT_DOUBLE_EQ(values[k], valuesOther[k]);
        ++k;
      }
    }
    for (; k < nnzJac; ++k) {
      EXPECT_DOUBLE_EQ(values[k], 1.0);
    }

    std::vector<double> residuals;
    if (useResidual) {
      residuals = {0.3, -0.7};
    }
    checkStaticOptimJacobian(
        problem, staticOptimTestPoint(activations, residuals), 1e-6);
  }

#endif
}

#endif
