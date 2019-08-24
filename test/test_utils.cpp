#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "Utils/String.h"
#include "Utils/Node3d.h"
#include "Utils/RotoTransNode.h"

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

