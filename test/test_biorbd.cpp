#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "ModelWriter.h"
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"

static double requiredPrecision(1e-10);

static std::string modelPathWithMeshFile("models/simpleWithMeshFile.bioMod");
#ifdef MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
static std::string modelFreeFall("models/pyomecaman_freeFall.bioMod");

static std::string modelPathWithObj("models/violin.bioMod");
#ifdef MODULE_VTP_FILES_READER
static std::string modelPathWithVtp("models/thoraxWithVtp.bioMod");
#endif

TEST(FileIO, OpenModel){
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(FileIO, WriteModel){
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::utils::String savePath("temporary.bioMod");
    biorbd::Writer::writeModel(model, savePath);
    biorbd::Model modelCopy(savePath);
    remove(savePath.c_str());
}
#endif

TEST(GenericTests, mass){
    biorbd::Model model(modelPathForGeneralTesting);
    SCALAR_TO_DOUBLE(mass, model.mass());
    EXPECT_NEAR(mass, 52.41212, requiredPrecision);
}

TEST(MeshFile, FileIO){
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithMeshFile));
    biorbd::Model model(modelPathWithMeshFile);
}

TEST(MeshFile, FileIoObj) {
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithObj));
    biorbd::Model model(modelPathWithObj);
}

#ifdef MODULE_VTP_FILES_READER
TEST(MeshFile, FileIoVtp) {
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithVtp));
    biorbd::Model model(modelPathWithVtp);
}
#endif
