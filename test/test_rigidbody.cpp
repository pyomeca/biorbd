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

#ifdef MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
TEST(Bone, copy)
{
    biorbd::Model model(modelPathForGeneralTesting);
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

TEST(Dynamics, Forward){
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model), QDot(model), QDDot(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    QDDot_expected << 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Tau.setZero();

    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot[i], QDDot_expected[i], 1e-6);
}

static std::string modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
TEST(Dynamics, ForwardLoopConstraint){
    biorbd::Model model(modelPathForLoopConstraintTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model), QDot(model), QDDot_constrained(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Q.setZero();
    QDot.setZero();
    QDDot_constrained.setZero();
    QDDot_expected << 0.35935119225397999,  -10.110263945851218,  0, -27.780105329642453, 31.783042765424835,
            44.636461505611521, 0.67809701484785911, 0, 0, 15.822263679783546, 0, 0;
    Tau.setZero();

    biorbd::rigidbody::Contacts& cs(model.getConstraints_nonConst());
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(model, Q, QDot, Tau, cs, QDDot_constrained);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot_constrained[i], QDDot_expected[i], 1e-6);
}
