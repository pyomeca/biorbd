#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "Utils/String.h"
#include "Utils/Path.h"
#include "Utils/Node3d.h"
#include "Utils/RotoTrans.h"
#include "Utils/RotoTransNode.h"
#include "RigidBody/GeneralizedCoordinates.h"

static double requiredPrecision(1e-10);

TEST(ShallowCopy, DeepCopy){
    // DeepCopying a shallow copy, should also change the reference
    // Warning that may be surprising because one may be tend to DeepCopy
    // itself afterward, this doesn't release the shallowcopy referencing
    biorbd::utils::Node3d MainNode(0, 0, 0, "NoName", "NoParent");
    biorbd::utils::Node3d ShallowToDeep(MainNode);
    biorbd::utils::Node3d NewNode(0, 0, 0, "MyName", "MyParent");
    EXPECT_STREQ(MainNode.name().c_str(), "NoName");
    EXPECT_STREQ(ShallowToDeep.name().c_str(), "NoName");
    EXPECT_STREQ(NewNode.name().c_str(), "MyName");
    ShallowToDeep.setName("StillNoName");
    EXPECT_STREQ(MainNode.name().c_str(), "StillNoName");
    EXPECT_STREQ(ShallowToDeep.name().c_str(), "StillNoName");
    EXPECT_STREQ(NewNode.name().c_str(), "MyName");
    ShallowToDeep.DeepCopy(NewNode);
    EXPECT_STREQ(MainNode.name().c_str(), "MyName");
    EXPECT_STREQ(ShallowToDeep.name().c_str(), "MyName");
    EXPECT_STREQ(NewNode.name().c_str(), "MyName");
    ShallowToDeep.setName("BackToNoName");
    EXPECT_STREQ(MainNode.name().c_str(), "BackToNoName");
    EXPECT_STREQ(ShallowToDeep.name().c_str(), "BackToNoName");
    EXPECT_STREQ(NewNode.name().c_str(), "MyName");
}

TEST(Path, Create){
    {
        biorbd::utils::Path emptyPath;
        EXPECT_STREQ(emptyPath.absolutePath().c_str(), biorbd::utils::Path::currentDir().c_str());
        EXPECT_STREQ(emptyPath.filename().c_str(), "");
        EXPECT_STREQ(emptyPath.extension().c_str(), "");
        EXPECT_STREQ(emptyPath.originalPath().c_str(), "");
    }

    {
#ifdef _WIN32
        biorbd::utils::String path("C:/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
#else
        biorbd::utils::String path("/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
#endif
        biorbd::utils::Path absolutePath(path);
        EXPECT_STREQ(absolutePath.absolutePath().c_str(), path.c_str());
        EXPECT_STREQ(absolutePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(absolutePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(absolutePath.originalPath().c_str(), path.c_str());
    }

    {
        biorbd::utils::Path relativePath("MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(relativePath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyLovely/RelativePath/ToMyLovelyFile.biorbd").c_str());
        EXPECT_STREQ(relativePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(relativePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(relativePath.relativePath().c_str(), "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(relativePath.originalPath().c_str(), "MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    }

    {
        biorbd::utils::Path weirdRelativePath("./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(weirdRelativePath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyLovely/RelativePath/ToMyLovelyFile.biorbd").c_str());
        EXPECT_STREQ(weirdRelativePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(weirdRelativePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(weirdRelativePath.relativePath().c_str(), "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(weirdRelativePath.originalPath().c_str(), "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    }

    {
        biorbd::utils::Path parentRelativePath("../MyLovely/ParentPath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(parentRelativePath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "../MyLovely/ParentPath/ToMyLovelyFile.biorbd").c_str());
        EXPECT_STREQ(parentRelativePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(parentRelativePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(parentRelativePath.relativePath().c_str(), "../MyLovely/ParentPath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(parentRelativePath.originalPath().c_str(), "../MyLovely/ParentPath/ToMyLovelyFile.biorbd");
    }

    {
        biorbd::utils::Path noPath("MyLonelyFile.biorbd");
        EXPECT_STREQ(noPath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyLonelyFile.biorbd").c_str());
        EXPECT_STREQ(noPath.filename().c_str(), "MyLonelyFile");
        EXPECT_STREQ(noPath.extension().c_str(), "biorbd");
        EXPECT_STREQ(noPath.relativePath().c_str(), "./MyLonelyFile.biorbd");
        EXPECT_STREQ(noPath.originalPath().c_str(), "MyLonelyFile.biorbd");
    }

    {
        biorbd::utils::Path almostNoPath("./MyKindOfLonelyFile.biorbd");
        EXPECT_STREQ(almostNoPath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyKindOfLonelyFile.biorbd").c_str());
        EXPECT_STREQ(almostNoPath.filename().c_str(), "MyKindOfLonelyFile");
        EXPECT_STREQ(almostNoPath.extension().c_str(), "biorbd");
        EXPECT_STREQ(almostNoPath.relativePath().c_str(), "./MyKindOfLonelyFile.biorbd");
        EXPECT_STREQ(almostNoPath.originalPath().c_str(), "./MyKindOfLonelyFile.biorbd");
    }
}

TEST(Path, Copy){
    biorbd::utils::Path MainPath("MyLovelyPath.biorbd");
    biorbd::utils::Path ShallowCopy(MainPath);
    biorbd::utils::Path DeepCopyNow(MainPath.DeepCopy());
    biorbd::utils::Path DeepCopyLater;
    DeepCopyLater.DeepCopy(MainPath);

    EXPECT_STREQ(MainPath.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyNow.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(MainPath.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyNow.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.originalPath().c_str(), "MyLovelyPath.biorbd");

    // Changing the ShallowCopy should change the Main, but not the Deep
    ShallowCopy.setFilename("MySecondLovelyPath");
    ShallowCopy.setExtension("newExt");

    EXPECT_STREQ(MainPath.relativePath().c_str(), "./MySecondLovelyPath.newExt");
    EXPECT_STREQ(ShallowCopy.relativePath().c_str(), "./MySecondLovelyPath.newExt");
    EXPECT_STREQ(DeepCopyNow.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(MainPath.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyNow.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.originalPath().c_str(), "MyLovelyPath.biorbd");
}

TEST(Node3d, rotate)
{
    biorbd::utils::Node3d node(2, 3, 4);
    biorbd::utils::Node3d nodeXrot(node.DeepCopy());
    nodeXrot.applyRT( biorbd::utils::RotoTrans(Eigen::Vector3d(M_PI, 0, 0), Eigen::Vector3d(0, 0, 0), "xyz") );
    EXPECT_NEAR(nodeXrot[0], node[0], requiredPrecision);
    EXPECT_NEAR(nodeXrot[1], -node[1], requiredPrecision);
    EXPECT_NEAR(nodeXrot[2], -node[2], requiredPrecision);

    biorbd::utils::Node3d nodeYrot(node.DeepCopy());
    nodeYrot.applyRT( biorbd::utils::RotoTrans(Eigen::Vector3d(0, M_PI, 0), Eigen::Vector3d(0, 0, 0), "xyz") );
    EXPECT_NEAR(nodeYrot[0], -node[0], requiredPrecision);
    EXPECT_NEAR(nodeYrot[1], node[1], requiredPrecision);
    EXPECT_NEAR(nodeYrot[2], -node[2], requiredPrecision);

    biorbd::utils::Node3d nodeZrot(node.DeepCopy());
    nodeZrot.applyRT( biorbd::utils::RotoTrans(Eigen::Vector3d(0, 0, M_PI), Eigen::Vector3d(0, 0, 0), "xyz") );
    EXPECT_NEAR(nodeZrot[0], -node[0], requiredPrecision);
    EXPECT_NEAR(nodeZrot[1], -node[1], requiredPrecision);
    EXPECT_NEAR(nodeZrot[2], node[2], requiredPrecision);

    biorbd::utils::Node3d nodeZrot2(node.DeepCopy());
    nodeZrot2.applyRT( biorbd::utils::RotoTrans(Eigen::Vector3d(M_PI, 0, 0), Eigen::Vector3d(0, 0, 0), "zxy") );
    EXPECT_NEAR(nodeZrot2[0], -node[0], requiredPrecision);
    EXPECT_NEAR(nodeZrot2[1], -node[1], requiredPrecision);
    EXPECT_NEAR(nodeZrot2[2], node[2], requiredPrecision);

    double trans(2);
    biorbd::utils::Node3d nodeRot(node.DeepCopy());
    nodeRot.applyRT( biorbd::utils::RotoTrans(Eigen::Vector3d(M_PI/6, M_PI/6, M_PI/6), Eigen::Vector3d(trans, trans, trans), "xyz") );
    EXPECT_NEAR(nodeRot[0], 4.200961894323342, requiredPrecision);
    EXPECT_NEAR(nodeRot[1], 3.4419872981077808, requiredPrecision);
    EXPECT_NEAR(nodeRot[2], 6.698557158514987, requiredPrecision);

    biorbd::utils::Node3d nodeTrans(node.DeepCopy());
    nodeTrans.applyRT( biorbd::utils::RotoTrans(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2), "zxy") );
    EXPECT_NEAR(nodeTrans[0], node[0] + trans, requiredPrecision);
    EXPECT_NEAR(nodeTrans[1], node[1] + trans, requiredPrecision);
    EXPECT_NEAR(nodeTrans[2], node[2] + trans, requiredPrecision);

}

TEST(Node3d, Copy){
    biorbd::utils::Node3d MainNode(1, 2, 3, "MainNodeName", "NoParent");
    biorbd::utils::Node3d ShallowCopy(MainNode);
    biorbd::utils::Node3d DeepCopyNow(MainNode.DeepCopy());
    biorbd::utils::Node3d DeepCopyLater;
    DeepCopyLater.DeepCopy(MainNode);

    EXPECT_STREQ(MainNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the values
    EXPECT_NEAR(MainNode.x(), 1, requiredPrecision);
    EXPECT_NEAR(MainNode.y(), 2, requiredPrecision);
    EXPECT_NEAR(MainNode.z(), 3, requiredPrecision);
    EXPECT_NEAR(ShallowCopy.x(), 1, requiredPrecision);
    EXPECT_NEAR(ShallowCopy.y(), 2, requiredPrecision);
    EXPECT_NEAR(ShallowCopy.z(), 3, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow.x(), 1, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow.y(), 2, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow.z(), 3, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater.x(), 1, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater.y(), 2, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater.z(), 3, requiredPrecision);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_NEAR(MainNode.x(), 1, requiredPrecision);
    EXPECT_NEAR(MainNode.y(), 2, requiredPrecision);
    EXPECT_NEAR(MainNode.z(), 3, requiredPrecision);
    EXPECT_NEAR(ShallowCopy.x(), 0, requiredPrecision);
    EXPECT_NEAR(ShallowCopy.y(), 0, requiredPrecision);
    EXPECT_NEAR(ShallowCopy.z(), 0, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow.x(), 1, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow.y(), 2, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow.z(), 3, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater.x(), 1, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater.y(), 2, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater.z(), 3, requiredPrecision);
}

TEST(Matrix, Copy){
    Eigen::Matrix4d tp;
    tp << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
    biorbd::utils::RotoTransNode MainRotoTransNode(tp, "NoName", "NoParent");
    biorbd::utils::RotoTransNode ShallowCopy(MainRotoTransNode);
    biorbd::utils::RotoTransNode DeepCopyNow(MainRotoTransNode.DeepCopy());
    biorbd::utils::RotoTransNode DeepCopyLater;
    DeepCopyLater.DeepCopy(MainRotoTransNode);

    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the values
    EXPECT_NEAR(MainRotoTransNode(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(ShallowCopy(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater(2, 2), 10, requiredPrecision);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_NEAR(MainRotoTransNode(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(ShallowCopy(2, 2), 0, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater(2, 2), 10, requiredPrecision);
}

TEST(RotoTransNode, Copy){
    Eigen::Matrix4d tp;
    tp << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
    biorbd::utils::RotoTransNode MainRotoTransNode(tp, "NoName", "NoParent");
    biorbd::utils::RotoTransNode ShallowCopy(MainRotoTransNode);
    biorbd::utils::RotoTransNode DeepCopyNow(MainRotoTransNode.DeepCopy());
    biorbd::utils::RotoTransNode DeepCopyLater;
    DeepCopyLater.DeepCopy(MainRotoTransNode);

    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the values
    EXPECT_NEAR(MainRotoTransNode(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(ShallowCopy(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater(2, 2), 10, requiredPrecision);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_NEAR(MainRotoTransNode(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(ShallowCopy(2, 2), 0, requiredPrecision);
    EXPECT_NEAR(DeepCopyNow(2, 2), 10, requiredPrecision);
    EXPECT_NEAR(DeepCopyLater(2, 2), 10, requiredPrecision);
}

TEST(ModelReading, equations)
{
    // The equation model was built so the x coordinates of the meshes should
    // be evaluated to the y coordinates.

    biorbd::Model m("models/equations.bioMod");
    std::vector<biorbd::utils::Node3d> mesh(
                m.meshPoints(biorbd::rigidbody::GeneralizedCoordinates(m).setZero(), 0, true));
    for (auto node : mesh)
        EXPECT_DOUBLE_EQ(node.x(), node.y());
}

