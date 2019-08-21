#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/BoneMesh.h"

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

