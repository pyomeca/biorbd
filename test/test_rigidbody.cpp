#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/BoneMesh.h"
#include "RigidBody/BoneCaracteristics.h"
#include "RigidBody/Bone.h"

static std::string modelPathForBone("models/pyomecaman.bioMod");
TEST(Bone, copy)
{
    biorbd::Model model(modelPathForBone);
    biorbd::rigidbody::BoneCaracteristics caract(10, biorbd::utils::Node3d(0.5, 0.5, 0.5), RigidBodyDynamics::Math::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
    biorbd::rigidbody::Bone MasterBone(model, "MasterBone", "NoParent", "zyx", "yzx", caract, RigidBodyDynamics::Math::SpatialTransform());
    biorbd::rigidbody::Bone ShallowCopy(MasterBone);
    biorbd::rigidbody::Bone DeepCopy(MasterBone.DeepCopy());

    EXPECT_STREQ(MasterBone.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");
    ShallowCopy.setParent("MyLovelyParent");
    EXPECT_STREQ(MasterBone.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");
}

TEST(BoneMesh, copy)
{
    biorbd::rigidbody::BoneMesh MasterMesh;
    MasterMesh.setPath("./MyFile.bioMesh");
    biorbd::rigidbody::BoneMesh ShallowCopy(MasterMesh);
    biorbd::rigidbody::BoneMesh DeepCopy(MasterMesh.DeepCopy());

    EXPECT_STREQ(MasterMesh.path().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopy.path().c_str(), "./MyFile.bioMesh");
    ShallowCopy.setPath("./MyNewFile.bioMesh");
    EXPECT_STREQ(MasterMesh.path().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(DeepCopy.path().c_str(), "./MyNewFile.bioMesh");
}

