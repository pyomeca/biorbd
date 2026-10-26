#include "biorbdConfig.h"

#include <gtest/gtest.h>
#include <iostream>

#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "ModelWriter.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/IMU.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/MeshFace.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "Utils/Range.h"
#include "Utils/RotoTrans.h"
#include "Utils/RotoTransNode.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

static double requiredPrecision(1e-10);

static std::string modelPathWithMeshFile("models/simpleWithMeshFile.bioMod");
#ifdef MODULE_ACTUATORS
static std::string modelPathForGeneralTesting(
    "models/pyomecaman_withActuators.bioMod");
#else   // MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif  // MODULE_ACTUATORS
static std::string modelFreeFall("models/pyomecaman_freeFall.bioMod");

static std::string modelPathWithObj("models/violin.bioMod");
#ifdef MODULE_VTP_FILES_READER
static std::string modelPathWithVtp("models/thoraxWithVtp.bioMod");
#endif
static std::string modelPathWithStl("models/pendulum.bioMod");
static std::string modelPathWithPly("models/simpleWithPlyMeshFile.bioMod");

TEST(FileIO, OpenModel) {
  EXPECT_NO_THROW(Model model(modelPathForGeneralTesting));
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(FileIO, WriteModel) {
  Model model("models/two_segments.bioMod");
  utils::String savePath("temporary.bioMod");
  Writer::writeModel(model, savePath);
  Model modelCopy(savePath);

  // Test if the model is properly written
  rigidbody::GeneralizedCoordinates Q(modelCopy);
  Q.setOnes();

  EXPECT_FLOAT_EQ(model.segment(0).jointDampings()[0], 1.0);
  EXPECT_FLOAT_EQ(model.segment(0).jointDampings()[1], 2.0);
  EXPECT_FLOAT_EQ(model.segment(0).jointDampings()[2], 3.0);
  EXPECT_FLOAT_EQ(model.segment(1).jointDampings()[0], 0.0);

  for (size_t k = 0; k < model.nbSegment(); ++k) {
    const std::vector<utils::Range>& QRanges = model.segment(k).QRanges();
    const std::vector<utils::Range>& QRangesCopy =
        modelCopy.segment(k).QRanges();
    ASSERT_EQ(QRangesCopy.size(), QRanges.size());
    for (size_t i = 0; i < QRanges.size(); ++i) {
      EXPECT_FLOAT_EQ(QRangesCopy[i].min(), QRanges[i].min());
      EXPECT_FLOAT_EQ(QRangesCopy[i].max(), QRanges[i].max());
    }

    const std::vector<utils::Range>& QdotRanges =
        model.segment(k).QdotRanges();
    const std::vector<utils::Range>& QdotRangesCopy =
        modelCopy.segment(k).QdotRanges();
    ASSERT_EQ(QdotRangesCopy.size(), QdotRanges.size());
    for (size_t i = 0; i < QdotRanges.size(); ++i) {
      EXPECT_FLOAT_EQ(QdotRangesCopy[i].min(), QdotRanges[i].min());
      EXPECT_FLOAT_EQ(QdotRangesCopy[i].max(), QdotRanges[i].max());
    }

    const std::vector<utils::Range>& QddotRanges =
        model.segment(k).QddotRanges();
    const std::vector<utils::Range>& QddotRangesCopy =
        modelCopy.segment(k).QddotRanges();
    ASSERT_EQ(QddotRangesCopy.size(), QddotRanges.size());
    for (size_t i = 0; i < QddotRanges.size(); ++i) {
      EXPECT_FLOAT_EQ(QddotRangesCopy[i].min(), QddotRanges[i].min());
      EXPECT_FLOAT_EQ(QddotRangesCopy[i].max(), QddotRanges[i].max());
    }
  }

  for (size_t k = 0; k < model.nbSegment(); ++k) {
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        EXPECT_NEAR(
            model.globalJCS(Q, k)(i, j), modelCopy.globalJCS(Q, k)(i, j), 1e-5);
      }
    }
  }

  EXPECT_EQ(modelCopy.nbMarkers(), 1);
  for (size_t k = 0; k < modelCopy.nbMarkers(); ++k) {
    for (size_t i = 0; i < 3; ++i) {
      EXPECT_NEAR(model.marker(Q, k)[i], modelCopy.marker(Q, k)[i], 1e-5);
    }
  }

  EXPECT_EQ(modelCopy.nbRTs(), 1);
  for (size_t k = 0; k < modelCopy.nbRTs(); ++k) {
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        EXPECT_NEAR(model.RT(Q, k)(i, j), modelCopy.RT(Q, k)(i, j), 1e-5);
      }
    }
  }

  EXPECT_EQ(modelCopy.nbIMUs(), 1);
  for (size_t k = 0; k < modelCopy.nbIMUs(); ++k) {
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        EXPECT_NEAR(model.IMU(Q)[k](i, j), modelCopy.IMU(Q)[k](i, j), 1e-5);
      }
    }
  }
  remove(savePath.c_str());
}
#endif

TEST(GenericTests, mass) {
  Model model(modelPathForGeneralTesting);
  SCALAR_TO_DOUBLE(mass, model.mass());
  EXPECT_NEAR(mass, 52.41212, requiredPrecision);
}

TEST(MeshFile, FileIO) {
  EXPECT_NO_THROW(Model model(modelPathWithMeshFile));
  Model model(modelPathWithMeshFile);
}

TEST(MeshFile, FileIoPly) {
  EXPECT_NO_THROW(Model model(modelPathWithPly));
  Model model(modelPathWithPly);
  EXPECT_EQ(model.mesh(0).nbVertex(), 8);
  EXPECT_EQ(model.mesh(0).faces().size(), 12);
}

#ifndef SKIP_LONG_TESTS
TEST(MeshFile, FileIoObj) {
  EXPECT_NO_THROW(Model model(modelPathWithObj));
  Model model(modelPathWithObj);
}
#endif

#ifdef MODULE_VTP_FILES_READER
TEST(MeshFile, FileIoVtp) { EXPECT_NO_THROW(Model model(modelPathWithVtp)); }
#endif

#ifndef SKIP_LONG_TESTS
TEST(MeshFile, FileIoStl) {
  EXPECT_NO_THROW(Model model(modelPathWithStl));
  Model model(modelPathWithStl);
}
#endif
