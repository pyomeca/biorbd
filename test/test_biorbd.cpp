#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"

static double requiredPrecision(1e-10);

static std::string modelPathWithMeshFile("models/simpleWithMeshFile.bioMod");
#ifdef MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS

TEST(FileIO, OpenModel){
    biorbd::Model model(modelPathForGeneralTesting);
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
}

TEST(GenericTests, mass){
    biorbd::Model model(modelPathForGeneralTesting);
    EXPECT_NEAR(model.mass(), 52.41212, requiredPrecision);
}

TEST(MeshFile, FileIO){
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithMeshFile));
    biorbd::Model model(modelPathWithMeshFile);
}

// TODO : Copy of a model with mesh file for the path
