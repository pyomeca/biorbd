#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "Utils/String.h"
#include "Utils/Path.h"
#include "Utils/Node3d.h"
#include "Utils/RotoTransNode.h"

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
    EXPECT_DOUBLE_EQ(MainNode.x(), 1);
    EXPECT_DOUBLE_EQ(MainNode.y(), 2);
    EXPECT_DOUBLE_EQ(MainNode.z(), 3);
    EXPECT_DOUBLE_EQ(ShallowCopy.x(), 1);
    EXPECT_DOUBLE_EQ(ShallowCopy.y(), 2);
    EXPECT_DOUBLE_EQ(ShallowCopy.z(), 3);
    EXPECT_DOUBLE_EQ(DeepCopyNow.x(), 1);
    EXPECT_DOUBLE_EQ(DeepCopyNow.y(), 2);
    EXPECT_DOUBLE_EQ(DeepCopyNow.z(), 3);
    EXPECT_DOUBLE_EQ(DeepCopyLater.x(), 1);
    EXPECT_DOUBLE_EQ(DeepCopyLater.y(), 2);
    EXPECT_DOUBLE_EQ(DeepCopyLater.z(), 3);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_DOUBLE_EQ(MainNode.x(), 1);
    EXPECT_DOUBLE_EQ(MainNode.y(), 2);
    EXPECT_DOUBLE_EQ(MainNode.z(), 3);
    EXPECT_DOUBLE_EQ(ShallowCopy.x(), 0);
    EXPECT_DOUBLE_EQ(ShallowCopy.y(), 0);
    EXPECT_DOUBLE_EQ(ShallowCopy.z(), 0);
    EXPECT_DOUBLE_EQ(DeepCopyNow.x(), 1);
    EXPECT_DOUBLE_EQ(DeepCopyNow.y(), 2);
    EXPECT_DOUBLE_EQ(DeepCopyNow.z(), 3);
    EXPECT_DOUBLE_EQ(DeepCopyLater.x(), 1);
    EXPECT_DOUBLE_EQ(DeepCopyLater.y(), 2);
    EXPECT_DOUBLE_EQ(DeepCopyLater.z(), 3);
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
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopyNow(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopyLater(2, 2), 10);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 0);
    EXPECT_DOUBLE_EQ(DeepCopyNow(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopyLater(2, 2), 10);
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
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopyNow(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopyLater(2, 2), 10);
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    EXPECT_DOUBLE_EQ(MainRotoTransNode(2, 2), 10);
    EXPECT_DOUBLE_EQ(ShallowCopy(2, 2), 0);
    EXPECT_DOUBLE_EQ(DeepCopyNow(2, 2), 10);
    EXPECT_DOUBLE_EQ(DeepCopyLater(2, 2), 10);
}

