#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "Utils/String.h"
#include "Utils/Path.h"
#include "Utils/Node3d.h"
#include "Utils/RotoTransNode.h"

TEST(Path, Create){

    biorbd::utils::Path absolutePath("/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
    EXPECT_STREQ(absolutePath.absolutePath().c_str(), "/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
    EXPECT_STREQ(absolutePath.filename().c_str(), "ToMyLovelyFile");
    EXPECT_STREQ(absolutePath.extension().c_str(), "biorbd");
    EXPECT_STREQ(absolutePath.originalPath().c_str(), "/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");

    biorbd::utils::Path relativePath("MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    EXPECT_STREQ(relativePath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyLovely/RelativePath/ToMyLovelyFile.biorbd").c_str());
    EXPECT_STREQ(relativePath.filename().c_str(), "ToMyLovelyFile");
    EXPECT_STREQ(relativePath.extension().c_str(), "biorbd");
    EXPECT_STREQ(relativePath.relativePath().c_str(), "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    EXPECT_STREQ(relativePath.originalPath().c_str(), "MyLovely/RelativePath/ToMyLovelyFile.biorbd");

    biorbd::utils::Path weirdRelativePath("./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    EXPECT_STREQ(weirdRelativePath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyLovely/RelativePath/ToMyLovelyFile.biorbd").c_str());
    EXPECT_STREQ(weirdRelativePath.filename().c_str(), "ToMyLovelyFile");
    EXPECT_STREQ(weirdRelativePath.extension().c_str(), "biorbd");
    EXPECT_STREQ(weirdRelativePath.relativePath().c_str(), "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    EXPECT_STREQ(weirdRelativePath.originalPath().c_str(), "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");

    biorbd::utils::Path noPath("MyLonelyFile.biorbd");
    EXPECT_STREQ(noPath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyLonelyFile.biorbd").c_str());
    EXPECT_STREQ(noPath.filename().c_str(), "MyLonelyFile");
    EXPECT_STREQ(noPath.extension().c_str(), "biorbd");
    EXPECT_STREQ(noPath.relativePath().c_str(), "./MyLonelyFile.biorbd");
    EXPECT_STREQ(noPath.originalPath().c_str(), "MyLonelyFile.biorbd");

    biorbd::utils::Path almostNoPath("./MyKindOfLonelyFile.biorbd");
    EXPECT_STREQ(almostNoPath.absolutePath().c_str(), (biorbd::utils::Path::currentDir() + "MyKindOfLonelyFile.biorbd").c_str());
    EXPECT_STREQ(almostNoPath.filename().c_str(), "MyKindOfLonelyFile");
    EXPECT_STREQ(almostNoPath.extension().c_str(), "biorbd");
    EXPECT_STREQ(almostNoPath.relativePath().c_str(), "./MyKindOfLonelyFile.biorbd");
    EXPECT_STREQ(almostNoPath.originalPath().c_str(), "./MyKindOfLonelyFile.biorbd");
}

TEST(Path, Copy){
    biorbd::utils::Path MainPath("MyLovelyPath.biorbd");
    biorbd::utils::Path ShallowCopy(MainPath);
    biorbd::utils::Path DeepCopy(MainPath.DeepCopy());

    EXPECT_STREQ(MainPath.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopy.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(MainPath.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopy.originalPath().c_str(), "MyLovelyPath.biorbd");

    // Changing the ShallowCopy should change the Main, but not the Deep
    ShallowCopy.setFilename("MySecondLovelyPath");
    ShallowCopy.setExtension("newExt");

    EXPECT_STREQ(MainPath.relativePath().c_str(), "./MySecondLovelyPath.newExt");
    EXPECT_STREQ(ShallowCopy.relativePath().c_str(), "./MySecondLovelyPath.newExt");
    EXPECT_STREQ(DeepCopy.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(MainPath.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopy.originalPath().c_str(), "MyLovelyPath.biorbd");
}


TEST(Node3d, Copy){
    biorbd::utils::Node3d MainNode(1, 2, 3, "MainNodeName", "NoParent");
    biorbd::utils::Node3d ShallowCopy(MainNode);
    biorbd::utils::Node3d DeepCopy(MainNode.DeepCopy());

    EXPECT_STREQ(MainNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");

    // Test for the values
    EXPECT_DOUBLE_EQ(MainNode.x(), 1);
    EXPECT_DOUBLE_EQ(MainNode.y(), 2);
    EXPECT_DOUBLE_EQ(MainNode.z(), 3);
    EXPECT_DOUBLE_EQ(ShallowCopy.x(), 1);
    EXPECT_DOUBLE_EQ(ShallowCopy.y(), 2);
    EXPECT_DOUBLE_EQ(ShallowCopy.z(), 3);
    EXPECT_DOUBLE_EQ(DeepCopy.x(), 1);
    EXPECT_DOUBLE_EQ(DeepCopy.y(), 2);
    EXPECT_DOUBLE_EQ(DeepCopy.z(), 3);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_DOUBLE_EQ(MainNode.x(), 1);
    EXPECT_DOUBLE_EQ(MainNode.y(), 2);
    EXPECT_DOUBLE_EQ(MainNode.z(), 3);
    EXPECT_DOUBLE_EQ(ShallowCopy.x(), 0);
    EXPECT_DOUBLE_EQ(ShallowCopy.y(), 0);
    EXPECT_DOUBLE_EQ(ShallowCopy.z(), 0);
    EXPECT_DOUBLE_EQ(DeepCopy.x(), 1);
    EXPECT_DOUBLE_EQ(DeepCopy.y(), 2);
    EXPECT_DOUBLE_EQ(DeepCopy.z(), 3);
}

TEST(Matrix, Copy){
    Eigen::Matrix4d tp;
    tp << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
    biorbd::utils::RotoTransNode MainRotoTransNode(tp, "NoName", "NoParent");
    biorbd::utils::RotoTransNode ShallowCopy(MainRotoTransNode);
    biorbd::utils::RotoTransNode DeepCopy(MainRotoTransNode.DeepCopy());

    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");

    // Test for the values
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopy(2, 2), 10);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 0);
    EXPECT_DOUBLE_EQ(DeepCopy(2, 2), 10);
}

TEST(RotoTransNode, Copy){
    Eigen::Matrix4d tp;
    tp << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
    biorbd::utils::RotoTransNode MainRotoTransNode(tp, "NoName", "NoParent");
    biorbd::utils::RotoTransNode ShallowCopy(MainRotoTransNode);
    biorbd::utils::RotoTransNode DeepCopy(MainRotoTransNode.DeepCopy());

    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopy.parent().c_str(), "NoParent");

    // Test for the values
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopy(2, 2), 10);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 0);
    EXPECT_DOUBLE_EQ(DeepCopy(2, 2), 10);
}

