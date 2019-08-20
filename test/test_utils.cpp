#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "Utils/String.h"
#include "Utils/Node3d.h"

TEST(Node, Copy){
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

    // Value are NOT shallow copy, therefore the parent should keep its values
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
