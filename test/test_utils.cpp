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
#include "Utils/Quaternion.h"

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
        EXPECT_STREQ(emptyPath.absolutePath().c_str(),
                     biorbd::utils::Path::currentDir().c_str());
        EXPECT_STREQ(emptyPath.filename().c_str(), "");
        EXPECT_STREQ(emptyPath.extension().c_str(), "");
        EXPECT_STREQ(emptyPath.originalPath().c_str(), "");
    }

    {
        EXPECT_STREQ(
            biorbd::utils::Path::toUnixFormat(
                            "MyUgly\\\\WindowsPath\\\\ToMyFile.biorbd").c_str(),
                    biorbd::utils::String (
                            "MyUgly/WindowsPath/ToMyFile.biorbd").c_str());

        EXPECT_STREQ(
            biorbd::utils::Path::toWindowsFormat(
                            "MyCute/UnixPath/ToMyFile.biorbd").c_str(),
                    biorbd::utils::String (
                            "MyCute\\\\UnixPath\\\\ToMyFile.biorbd").c_str());
    }

    {
        biorbd::utils::String myPathInUglyWindowsStyleString(
                    "MyUgly\\\\WindowsPath\\\\ToMyFile.biorbd");
        biorbd::utils::Path myPathInUglyWindowsStyle(
                    myPathInUglyWindowsStyleString);
        EXPECT_STREQ(myPathInUglyWindowsStyle.relativePath().c_str(),
                     "./MyUgly/WindowsPath/ToMyFile.biorbd");
        EXPECT_STREQ(myPathInUglyWindowsStyle.originalPath().c_str(),
                     myPathInUglyWindowsStyleString.c_str());
    }

    {
#ifdef _WIN32
        biorbd::utils::String path(
                    "C:\\MyLovely\\AbsolutePath\\ToMyLovelyFile.biorbd");
        biorbd::utils::String unixPath(
                    "C:/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
#else
        biorbd::utils::String path(
                    "/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
        biorbd::utils::String unixPath(
                    "/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
#endif

        biorbd::utils::Path absolutePath(path);
        EXPECT_STREQ(absolutePath.absolutePath().c_str(), unixPath.c_str());
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
    nodeTrans.applyRT( biorbd::utils::RotoTrans(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2), "xyz") );
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

TEST(RotoTrans, unitTest){
    biorbd::utils::RotoTrans rt(
                Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1), "xyz");
    Eigen::Matrix4d rtExpected;
    rtExpected <<
            0.29192658172642888, -0.45464871341284091,  0.84147098480789650, 1,
            0.83722241402998721, -0.30389665486452672, -0.45464871341284091, 1,
            0.46242567005663016,  0.83722241402998732,  0.29192658172642888, 1,
            0, 0, 0, 1;
    for (unsigned int i = 0; i<4; ++i){
        for (unsigned int j = 0; j<4; ++j){
            EXPECT_NEAR(rt(i, j), rtExpected(i,j), requiredPrecision);
        }
    }

    biorbd::utils::RotoTrans rt_t(rt.transpose());
    Eigen::Matrix4d rtTransposedExpected;
    rtTransposedExpected <<
            0.29192658172642888,  0.83722241402998721, 0.46242567005663016, -1.5915746658130461,
           -0.45464871341284091, -0.30389665486452672, 0.83722241402998732, -0.07867704575261969,
            0.84147098480789650, -0.45464871341284091, 0.29192658172642888, -0.67874885312148447,
            0, 0, 0, 1;
    for (unsigned int i = 0; i<4; ++i){
        for (unsigned int j = 0; j<4; ++j){
            EXPECT_NEAR(rt_t(i, j), rtTransposedExpected(i,j),
                        requiredPrecision);
        }
    }

    biorbd::utils::Node3d marker(1, 1, 1);
    marker.applyRT(rt_t);
    for (unsigned int i=0; i<3; ++i)
        EXPECT_NEAR(marker[i], 0, requiredPrecision);
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

TEST(Quaternion, creation)
{
    {
        biorbd::utils::Quaternion quat;
        EXPECT_NEAR(quat.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat.x(), 0, requiredPrecision);
        EXPECT_NEAR(quat.y(), 0, requiredPrecision);
        EXPECT_NEAR(quat.z(), 0, requiredPrecision);
        EXPECT_NEAR(quat.kStab(), 1, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion quat(1,2,3,4);
        EXPECT_NEAR(quat.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat.z(), 4, requiredPrecision);
        EXPECT_NEAR(quat[0], 1, requiredPrecision);
        EXPECT_NEAR(quat[1], 2, requiredPrecision);
        EXPECT_NEAR(quat[2], 3, requiredPrecision);
        EXPECT_NEAR(quat[3], 4, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion quat(Eigen::Vector4d(1,2,3,4));
        EXPECT_NEAR(quat.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat.z(), 4, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion quat(1, Eigen::Vector3d(2,3,4));
        EXPECT_NEAR(quat.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat.z(), 4, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion quat1(1,2,3,4,5);
        biorbd::utils::Quaternion quat2(quat1);
        biorbd::utils::Quaternion quat3 = quat1;

        EXPECT_NEAR(quat2.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat2.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat2.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat2.z(), 4, requiredPrecision);
        EXPECT_NEAR(quat2.kStab(), 5, requiredPrecision);

        EXPECT_NEAR(quat3.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat3.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat3.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat3.z(), 4, requiredPrecision);
        EXPECT_NEAR(quat3.kStab(), 5, requiredPrecision);
    }
    {
        Eigen::Vector4d quat1(1,2,3,4);
        biorbd::utils::Quaternion quat2(quat1);
        biorbd::utils::Quaternion quat3;
        quat3 = quat1;

        EXPECT_NEAR(quat2.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat2.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat2.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat2.z(), 4, requiredPrecision);
        EXPECT_NEAR(quat2.kStab(), 1, requiredPrecision);

        EXPECT_NEAR(quat3.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat3.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat3.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat3.z(), 4, requiredPrecision);
        EXPECT_NEAR(quat3.kStab(), 1, requiredPrecision);
    }
}

TEST(Quaternion, addition)
{
    biorbd::utils::Quaternion q1(1,2,3,4,2);
    biorbd::utils::Quaternion q2(2,3,4,5,3);
    biorbd::utils::Quaternion q12 = q1+q2;

    EXPECT_NEAR(q12.w(), 3, requiredPrecision);
    EXPECT_NEAR(q12.x(), 5, requiredPrecision);
    EXPECT_NEAR(q12.y(), 7, requiredPrecision);
    EXPECT_NEAR(q12.z(), 9, requiredPrecision);
    EXPECT_NEAR(q12.kStab(), 2.5, requiredPrecision);
}


TEST(Quaternion, multiplication)
{
    biorbd::utils::Quaternion q1(1,2,3,4,2);
    biorbd::utils::Quaternion q2(2,3,4,5,3);
    double d(5);
    float f(5);

    biorbd::utils::Quaternion q12 = q1*q2;
    EXPECT_NEAR(q12.w(), -36, requiredPrecision);
    EXPECT_NEAR(q12.x(), 6, requiredPrecision);
    EXPECT_NEAR(q12.y(), 12, requiredPrecision);
    EXPECT_NEAR(q12.z(), 12, requiredPrecision);
    EXPECT_NEAR(q12.kStab(), 2.5, requiredPrecision);

    biorbd::utils::Quaternion q1d = q1*d;
    EXPECT_NEAR(q1d.w(), 5, requiredPrecision);
    EXPECT_NEAR(q1d.x(), 10, requiredPrecision);
    EXPECT_NEAR(q1d.y(), 15, requiredPrecision);
    EXPECT_NEAR(q1d.z(), 20, requiredPrecision);
    EXPECT_NEAR(q1d.kStab(), 2, requiredPrecision);

    biorbd::utils::Quaternion q1f = q1*f;
    EXPECT_NEAR(q1f.w(), 5, requiredPrecision);
    EXPECT_NEAR(q1f.x(), 10, requiredPrecision);
    EXPECT_NEAR(q1f.y(), 15, requiredPrecision);
    EXPECT_NEAR(q1f.z(), 20, requiredPrecision);
    EXPECT_NEAR(q1f.kStab(), 2, requiredPrecision);
}

TEST(Quaternion, conversion) {
    {
        biorbd::utils::Quaternion q(
                    biorbd::utils::Quaternion::fromGLRotate(1, 2, 3, 4, 5));
        EXPECT_NEAR(q.w(), 0.99996192306417131, requiredPrecision);
        EXPECT_NEAR(q.x(), 0.017453070996747869, requiredPrecision);
        EXPECT_NEAR(q.y(), 0.026179606495121806, requiredPrecision);
        EXPECT_NEAR(q.z(), 0.034906141993495739, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
                    biorbd::utils::Quaternion::fromAxisAngle(
                        1, biorbd::utils::Node3d(2, 3, 4), 5));
        EXPECT_NEAR(q.w(), 0.87758256189037276, requiredPrecision);
        EXPECT_NEAR(q.x(), 0.17805417504364543, requiredPrecision);
        EXPECT_NEAR(q.y(), 0.26708126256546816, requiredPrecision);
        EXPECT_NEAR(q.z(), 0.35610835008729086, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    biorbd::utils::RotoTrans rt(
                Eigen::Vector3d(2, 3, 4), Eigen::Vector3d(), "xyz");
    {
        biorbd::utils::Quaternion q(
                    biorbd::utils::Quaternion::fromMatrix(rt, 5));
        EXPECT_NEAR(q.w(), 0.77913560959923722, requiredPrecision);
        EXPECT_NEAR(q.x(), 0.46529436049374817, requiredPrecision);
        EXPECT_NEAR(q.y(), -0.27840624141687692, requiredPrecision);
        EXPECT_NEAR(q.z(), -0.31454542547502823, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
                    biorbd::utils::Quaternion::fromMatrix(rt.rot(), 5));
        EXPECT_NEAR(q.w(), 0.77913560959923722, requiredPrecision);
        EXPECT_NEAR(q.x(), 0.46529436049374817, requiredPrecision);
        EXPECT_NEAR(q.y(), -0.27840624141687692, requiredPrecision);
        EXPECT_NEAR(q.z(), -0.31454542547502823, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
                    biorbd::utils::Quaternion::fromZYXAngles(
                        biorbd::utils::Node3d(2, 3, 4), 5));
        EXPECT_NEAR(q.w(), 0.74732578388941828, requiredPrecision);
        EXPECT_NEAR(q.x(), 0.38405116269438366, requiredPrecision);
        EXPECT_NEAR(q.y(), -0.17015746936361908, requiredPrecision);
        EXPECT_NEAR(q.z(), -0.51483522877414645, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
                    biorbd::utils::Quaternion::fromYXZAngles(
                        biorbd::utils::Node3d(2, 3, 4), 5));
        EXPECT_NEAR(q.w(), 0.74732578388941828, requiredPrecision);
        EXPECT_NEAR(q.x(), -0.17015746936361908, requiredPrecision);
        EXPECT_NEAR(q.y(), -0.51483522877414645, requiredPrecision);
        EXPECT_NEAR(q.z(), 0.38405116269438366, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
                    biorbd::utils::Quaternion::fromXYZAngles(
                        biorbd::utils::Node3d(2, 3, 4), 5));
        EXPECT_NEAR(q.w(), 0.74732578388941839, requiredPrecision);
        EXPECT_NEAR(q.x(), -0.51483522877414645, requiredPrecision);
        EXPECT_NEAR(q.y(), -0.17015746936361908, requiredPrecision);
        EXPECT_NEAR(q.z(), 0.38405116269438366, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::RotoTrans rt(q.toMatrix());
        Eigen::Matrix3d mat(rt.rot());

        EXPECT_NEAR(mat(0, 0), -81, requiredPrecision);
        EXPECT_NEAR(mat(0, 1), 44, requiredPrecision);
        EXPECT_NEAR(mat(0, 2), 14, requiredPrecision);
        EXPECT_NEAR(mat(1, 0), 4, requiredPrecision);
        EXPECT_NEAR(mat(1, 1), -67, requiredPrecision);
        EXPECT_NEAR(mat(1, 2), 52, requiredPrecision);
        EXPECT_NEAR(mat(2, 0), 46, requiredPrecision);
        EXPECT_NEAR(mat(2, 1), 28, requiredPrecision);
        EXPECT_NEAR(mat(2, 2), -49, requiredPrecision);
    }
}

TEST(Quaternion, otherOperations)
{
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion q2(3, 4, 5, 6, 8);
        biorbd::utils::Quaternion qSlerp(q.slerp (7, q2));

        EXPECT_NEAR(qSlerp.w(), 8.582415805503274, requiredPrecision);
        EXPECT_NEAR(qSlerp.x(), 9.493173329713267, requiredPrecision);
        EXPECT_NEAR(qSlerp.y(), 10.40393085392326, requiredPrecision);
        EXPECT_NEAR(qSlerp.z(), 11.314688378133257, requiredPrecision);
        EXPECT_NEAR(qSlerp.kStab(), 7, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion qConj(q.conjugate());

        EXPECT_NEAR(qConj.w(), 2, requiredPrecision);
        EXPECT_NEAR(qConj.x(), -3, requiredPrecision);
        EXPECT_NEAR(qConj.y(), -4, requiredPrecision);
        EXPECT_NEAR(qConj.z(), -5, requiredPrecision);
        EXPECT_NEAR(qConj.kStab(), 6, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion qTime(
                    q.timeStep(biorbd::utils::Node3d(7, 8, 9), 0.1));

        EXPECT_NEAR(qTime.w(), -2.9791236033976602, requiredPrecision);
        EXPECT_NEAR(qTime.x(), 3.1304258212109395, requiredPrecision);
        EXPECT_NEAR(qTime.y(), 3.4370175820938655, requiredPrecision);
        EXPECT_NEAR(qTime.z(), 4.8489346122578709, requiredPrecision);
        EXPECT_NEAR(qTime.kStab(), 6, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Node3d vec(q.rotate(biorbd::utils::Node3d(7, 8, 9)));

        EXPECT_NEAR(vec.x(), 282, requiredPrecision);
        EXPECT_NEAR(vec.y(), 384, requiredPrecision);
        EXPECT_NEAR(vec.z(), 582, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion qdot(q.omegaToQDot(biorbd::utils::Node3d(7, 8, 9)));

        EXPECT_NEAR(qdot.w(), -49, requiredPrecision);
        EXPECT_NEAR(qdot.x(), 5, requiredPrecision);
        EXPECT_NEAR(qdot.y(), 12, requiredPrecision);
        EXPECT_NEAR(qdot.z(), 7, requiredPrecision);
        EXPECT_NEAR(qdot.kStab(), 6, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q1(1, 0, 0, 0, 5);
        biorbd::utils::Vector v(3);
        v << 1,2,3;
        q1.derivate(v);
        EXPECT_NEAR(q1.w(), 0, requiredPrecision);
        EXPECT_NEAR(q1.x(), 0.5, requiredPrecision);
        EXPECT_NEAR(q1.y(), 1, requiredPrecision);
        EXPECT_NEAR(q1.z(), 1.5, requiredPrecision);
        EXPECT_NEAR(q1.kStab(), 5, requiredPrecision);
        double w(0.07035975447302918);
        double x(0.7035975447302919);
        double y(0.7035975447302919);
        double z(0.07035975447302918);
        biorbd::utils::Quaternion q2(w,x,y,z, 5);
        q2.derivate(v);
        EXPECT_NEAR(q2.w(),-1.1609359488049815, requiredPrecision);
        EXPECT_NEAR(q2.x(), 1.0202164398589233, requiredPrecision);
        EXPECT_NEAR(q2.y(),-0.9498566853858941, requiredPrecision);
        EXPECT_NEAR(q2.z(), 0.45733840407468973, requiredPrecision);
        EXPECT_NEAR(q2.kStab(), 5, requiredPrecision);
    }
}
