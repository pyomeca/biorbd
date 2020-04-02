#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Utils/Range.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/IMU.h"
#ifndef SKIP_KALMAN
#include "RigidBody/KalmanReconsMarkers.h"
#include "RigidBody/KalmanReconsIMU.h"
#endif


static double requiredPrecision(1e-10);

#ifdef MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
static std::string modelPathMeshEqualsMarker("models/meshsEqualMarkers.bioMod");
static std::string modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
static std::string modelNoRoot("models/pyomecaman_freeFall.bioMod");
static std::string modelPathForImuTesting("models/pyomecaman_withIMUs.bioMod");

TEST(Contacts, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);
        EXPECT_STREQ(contacts.name(1).c_str(), "PiedG_1_Z");
        EXPECT_EQ(contacts.hasContacts(), true);
        EXPECT_NEAR(contacts.getForce()[1], 0., requiredPrecision);
    }
    {
        biorbd::rigidbody::Contacts contacts;
        EXPECT_EQ(contacts.hasContacts(), false);
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        EXPECT_THROW(contacts.name(7), std::runtime_error);
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);

        contacts.AddConstraint(
            7,
            biorbd::utils::Vector3d(0, 0, 0),
            biorbd::utils::Vector3d(1, 1, 1),
            "constraintName",
            2.0);

        EXPECT_NEAR(contacts.nbContacts(), 7., requiredPrecision);
    }
}

TEST(Contacts, DeepCopy)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        biorbd::rigidbody::Contacts shallowCopy(contacts);
        biorbd::rigidbody::Contacts deepCopyNow(contacts.DeepCopy());
        biorbd::rigidbody::Contacts deepCopyLater;
        deepCopyLater.DeepCopy(contacts);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(shallowCopy.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyNow.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyLater.nbContacts(), 6., requiredPrecision);

        contacts.AddConstraint(
            7,
            biorbd::utils::Vector3d(0, 0, 0),
            biorbd::utils::Vector3d(1, 1, 1),
            "constraintName",
            2.0);

        EXPECT_NEAR(contacts.nbContacts(), 7., requiredPrecision);
        EXPECT_NEAR(shallowCopy.nbContacts(), 7., requiredPrecision);
        EXPECT_NEAR(deepCopyNow.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyLater.nbContacts(), 6., requiredPrecision);
    }
}

static std::vector<double> Qtest = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                     0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3 };
TEST(GeneralizedCoordinates, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            Q[i] = Qtest[i];
        }

        biorbd::rigidbody::GeneralizedCoordinates newQ(Q);

        std::vector<double> Q_expected = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                     0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3 };

        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            EXPECT_NEAR(newQ[i], Q_expected[i], requiredPrecision);
        }
    }
}

TEST(GeneralizedVelocity, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedVelocity Qdot(model);
        for (unsigned int i = 0; i < model.nbQdot(); ++i) {
            Qdot[i] = Qtest[i] * 10;
        }

        std::vector<double> Qdot_expected = { 1., 1., 1., 3., 3., 3.,
                                     3., 3., 3., 3., 3., 4., 3.};

        biorbd::rigidbody::GeneralizedVelocity newQdot(Qdot);

        for (unsigned int i = 0; i < model.nbQdot(); ++i) {
            EXPECT_NEAR(newQdot[i], Qdot_expected[i], requiredPrecision);
        }
    }
    {
        biorbd::rigidbody::GeneralizedVelocity Qdot;
        EXPECT_NEAR(Qdot.norm(), 0., requiredPrecision);
   
        biorbd::rigidbody::GeneralizedVelocity newQdot(Qdot);
        EXPECT_NEAR(newQdot.norm(), 0., requiredPrecision);
    }
}

TEST(GeneralizedAcceleration, unitTest)
{
    {
        biorbd::rigidbody::GeneralizedAcceleration Qddot;
        EXPECT_NEAR(Qddot.norm(), 0., requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
        for (unsigned int i = 0; i < model.nbQ(); ++i){
            Qddot[i] = Qtest[i] * 100;
        }

        std::vector<double> Qddot_expected = { 10., 10., 10., 30., 30., 30.,
                             30., 30., 30., 30., 30., 40., 30. };

        biorbd::rigidbody::GeneralizedAcceleration newQddot(Qddot);

        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            EXPECT_NEAR(newQddot[i], Qddot_expected[i], requiredPrecision);
        }
    }
}

TEST(GeneralizedTorque, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedTorque Tau(model);
        Tau << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3; 

        std::vector<double> Tau_expected = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3 };

        for (unsigned int i = 0; i < 12; ++i) {
            EXPECT_NEAR(Tau[i], Tau_expected[i], requiredPrecision);
        }

        biorbd::rigidbody::GeneralizedTorque newTau(Tau);
        for (unsigned int i = 0; i < 12; ++i) {
            EXPECT_NEAR(newTau[i], Tau_expected[i], requiredPrecision);
        }

        biorbd::rigidbody::GeneralizedTorque tauWithNoArgument;
        EXPECT_NEAR(tauWithNoArgument.norm(), 0., requiredPrecision);
    }
}

TEST(IMU, unitTest)
{
    biorbd::rigidbody::IMU imu(true, false);
    EXPECT_EQ(imu.isTechnical(), true);
    EXPECT_EQ(imu.isAnatomical(), false);
}

TEST(IMU, DeepCopy)
{
    biorbd::rigidbody::IMU imu(false, true);

    biorbd::rigidbody::IMU shallowCopy(imu);
    biorbd::rigidbody::IMU deepCopyNow(imu.DeepCopy());
    biorbd::rigidbody::IMU deepCopyLater;
    deepCopyLater.DeepCopy(imu);

    EXPECT_EQ(shallowCopy.isTechnical(), false);
    EXPECT_EQ(deepCopyNow.isTechnical(), false);
    EXPECT_EQ(deepCopyLater.isTechnical(), false);
}

TEST(IMUs, unitTest)
{
    {
        biorbd::rigidbody::IMUs imus;
        imus.addIMU(true, true);

        EXPECT_EQ(imus.nbIMUs(), 1);
        EXPECT_EQ(imus.IMU(0).isTechnical(), true);
    }
    {
        biorbd::rigidbody::IMUs imus;
        biorbd::rigidbody::IMU imu(true, false);
        imu.setName("imuName");
        imus.addIMU(imu);

        EXPECT_STREQ(imus.technicalIMU()[0].name().c_str(), "imuName");
    }
    {
        biorbd::Model model(modelPathForImuTesting);
        biorbd::rigidbody::IMUs imus(model);

        EXPECT_NEAR(imus.anatomicalIMU().size(), 2., requiredPrecision);
        EXPECT_NEAR(imus.technicalIMU().size(), 4., requiredPrecision);
    }
}

TEST(IMUs, deepCopy)
{
    biorbd::Model model(modelPathForImuTesting);
    biorbd::rigidbody::IMUs imus(model);

    biorbd::rigidbody::IMUs shallowCopy(imus);
    biorbd::rigidbody::IMUs deepCopyNow(imus.DeepCopy());
    biorbd::rigidbody::IMUs deepCopyLater;
    deepCopyLater.DeepCopy(imus);

    EXPECT_EQ(shallowCopy.nbIMUs(), 4);
    EXPECT_EQ(deepCopyNow.nbIMUs(), 4);
    EXPECT_EQ(deepCopyLater.nbIMUs(), 4);

    imus.addIMU(true, true);
    EXPECT_EQ(imus.nbIMUs(), 5);
    EXPECT_EQ(shallowCopy.nbIMUs(), 5);
    EXPECT_EQ(deepCopyNow.nbIMUs(), 4);
    EXPECT_EQ(deepCopyLater.nbIMUs(), 4);
}

TEST(Joints, copy)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);

        biorbd::rigidbody::Joints shallowCopy(joints);
        biorbd::rigidbody::Joints deepCopyNow(joints.DeepCopy());
        biorbd::rigidbody::Joints deepCopyLater;
        deepCopyLater.DeepCopy(joints);

        EXPECT_NEAR(shallowCopy.mass(), 52.412120000000002, requiredPrecision);
        EXPECT_NEAR(deepCopyNow.mass(), 52.412120000000002, requiredPrecision);
        EXPECT_NEAR(deepCopyLater.mass(), 52.412120000000002, requiredPrecision);

        biorbd::rigidbody::SegmentCharacteristics characteristics(
            10, biorbd::utils::Vector3d(0.5, 0.5, 0.5),
            RigidBodyDynamics::Math::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
        std::vector<biorbd::utils::Range> ranges(6);
   
        joints.AddSegment("segmentName", "parentName", "zyx", "yzx", ranges,
            characteristics, RigidBodyDynamics::Math::SpatialTransform());

        EXPECT_NEAR(joints.mass(), 62.412120000000002, requiredPrecision);
        EXPECT_NEAR(shallowCopy.mass(), 62.412120000000002, requiredPrecision);
        EXPECT_NEAR(deepCopyNow.mass(), 52.412120000000002, requiredPrecision);
        EXPECT_NEAR(deepCopyLater.mass(), 52.412120000000002, requiredPrecision);
    }
}

TEST(Joints, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);
        std::vector <biorbd::utils::String> names(joints.nameDof());

        std::vector<biorbd::utils::String> expectedNames(joints.nbDof());
        expectedNames = { "Pelvis_TransY", "Pelvis_TransZ", "Pelvis_RotX",
            "BrasD_RotZ", "BrasD_RotX", "BrasG_RotZ", "BrasG_RotX",
            "CuisseD_RotX", "JambeD_RotX", "PiedD_RotX", "CuisseG_RotX",
            "JambeG_RotX", "PiedG_RotX" };


        for (int i = 0; i < joints.nbDof(); ++i) {
            EXPECT_STREQ(names[i].c_str(), expectedNames[i].c_str());
        }
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);
        biorbd::rigidbody::Segment segmentToTest(joints.segment("Tronc"));

        EXPECT_EQ(segmentToTest.id(), 2147483647);
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity Qdot(model);

        biorbd::utils::Vector3d angularMomentum(joints.angularMomentum(Q, Qdot));
        biorbd::utils::Vector3d expectedAngularMomentum(0., 0., 0.5);

        for (int i = 0; i < 2; ++i) {
            EXPECT_NEAR(angularMomentum[i], expectedAngularMomentum[i], requiredPrecision);
        }
    }
}

TEST(Markers, copy)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Markers markers(model);

        biorbd::rigidbody::Markers shallowCopy(markers);
        biorbd::rigidbody::Markers deepCopyNow(markers.DeepCopy());
        biorbd::rigidbody::Markers deepCopyLater;
        deepCopyLater.DeepCopy(markers);

        EXPECT_EQ(markers.nbMarkers(), 97);
        EXPECT_EQ(shallowCopy.nbMarkers(), 97);
        EXPECT_EQ(deepCopyNow.nbMarkers(), 97);
        EXPECT_EQ(deepCopyLater.nbMarkers(), 97);


        biorbd::rigidbody::NodeSegment nodeSegment;
        markers.addMarker(nodeSegment,
            "markerName", "parentName", true, true, "x", 98);

        EXPECT_EQ(markers.nbMarkers(), 98);
        EXPECT_EQ(shallowCopy.nbMarkers(), 98);
        EXPECT_EQ(deepCopyNow.nbMarkers(), 97);
        EXPECT_EQ(deepCopyLater.nbMarkers(), 97);
    }
}

TEST(SegmentCharacteristics, length)
{
    biorbd::rigidbody::SegmentCharacteristics segmentCharac;
    EXPECT_NEAR(segmentCharac.length(), 0., requiredPrecision);
    segmentCharac.setLength(2.);
    EXPECT_NEAR(segmentCharac.length(), 2., requiredPrecision);
}

TEST(Segment, nameDof)
{
    biorbd::Model model(modelPathForGeneralTesting);
    EXPECT_THROW(model.segment(128), std::runtime_error);
}

TEST(RotoTransNode, copy)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::RotoTransNodes rtNode(model);
    biorbd::utils::RotoTrans rt(
        biorbd::utils::Vector3d(2, 3, 4), biorbd::utils::Vector3d(), "xyz");
    rtNode.addRT(rt);

    biorbd::rigidbody::RotoTransNodes shallowCopy(rtNode);
    biorbd::rigidbody::RotoTransNodes deepCopyNow(rtNode.DeepCopy());
    biorbd::rigidbody::RotoTransNodes deepCopyLater;
    deepCopyLater.DeepCopy(rtNode);

    EXPECT_EQ(shallowCopy.nbRTs(), 1);
    EXPECT_EQ(deepCopyNow.nbRTs(), 1);
    EXPECT_EQ(deepCopyLater.nbRTs(), 1);

    rtNode.addRT(rt);
    EXPECT_EQ(rtNode.nbRTs(), 2);
    EXPECT_EQ(shallowCopy.nbRTs(), 2);
    EXPECT_EQ(deepCopyNow.nbRTs(), 1);
    EXPECT_EQ(deepCopyLater.nbRTs(), 1);
}

TEST(RotoTransNode, unitTest)
{
    {
        biorbd::rigidbody::RotoTransNodes rtNode;
        biorbd::utils::RotoTrans rt(
            biorbd::utils::Vector3d(2, 3, 4), biorbd::utils::Vector3d(), "xyz");
        rtNode.addRT(rt);
        auto rt_vector(rtNode.RTs());

        EXPECT_NEAR(rt_vector[0].norm(), 1.9999999999999998, requiredPrecision);
        EXPECT_NEAR(rtNode.RT(0).norm(), 1.9999999999999998, requiredPrecision);
    }
    {
        biorbd::rigidbody::RotoTransNodes rtNode;
        biorbd::utils::RotoTrans rt(
            biorbd::utils::Vector3d(2, 3, 4), biorbd::utils::Vector3d(), "xyz");
        rtNode.addRT(rt);
        auto rt_vector(rtNode.RTs());
        rt_vector[0].setParent("parentName");
        rt_vector[0].setName("nameSet");

        EXPECT_STREQ(rtNode.RTs("parentName")[0].name().c_str(), "nameSet");
        EXPECT_STREQ(rtNode.RTsNames()[0].c_str(), "nameSet");
    }
}

TEST(NodeSegment, unitTests)
{
    {
        biorbd::rigidbody::NodeSegment nodeSegment(1., 2., 3.);
        EXPECT_NEAR(nodeSegment.z(), 3., requiredPrecision);
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
            "nodeSegmentName", "parentName", true, true, "z", 8);
        EXPECT_STREQ(nodeSegment.parent().c_str(), "parentName");
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
            "nodeSegmentName", "parentName", true, true, "z", 8);

        EXPECT_EQ(nodeSegment.isAxisKept(2), false);
        EXPECT_EQ(nodeSegment.isAxisRemoved(2), true);
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(1., 2., 3.);
        EXPECT_THROW(nodeSegment.addAxesToRemove(4), std::runtime_error);
    }
}

TEST(NodeSegment, copy)
{
    biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
        "nodeSegmentName", "parentName", true, true, "z", 8);

    biorbd::rigidbody::NodeSegment deepCopyNow(nodeSegment.DeepCopy());
    biorbd::rigidbody::NodeSegment deepCopyLater;
    deepCopyLater.DeepCopy(nodeSegment);

    EXPECT_EQ(nodeSegment.nbAxesToRemove(), 1);
    EXPECT_EQ(deepCopyNow.nbAxesToRemove(), 1);
    EXPECT_EQ(deepCopyLater.nbAxesToRemove(), 1);
}
TEST(DegreesOfFreedom, count)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        EXPECT_EQ(model.nbQ(), 13);
        EXPECT_EQ(model.nbQdot(), 13);
        EXPECT_EQ(model.nbQddot(), 13);
        EXPECT_EQ(model.nbGeneralizedTorque(), 13);
        EXPECT_EQ(model.nbRoot(), 3);
    }
    {
        biorbd::Model model(modelNoRoot);
        EXPECT_EQ(model.nbQ(), 13);
        EXPECT_EQ(model.nbQdot(), 13);
        EXPECT_EQ(model.nbQddot(), 13);
        EXPECT_EQ(model.nbGeneralizedTorque(), 13);
        EXPECT_EQ(model.nbRoot(), 3);
    }
}

TEST(DegressOfFreedom, ranges) {
    biorbd::Model model(modelPathForGeneralTesting);
    std::vector<biorbd::utils::Range> ranges;

    auto a = model.meshPoints(biorbd::rigidbody::GeneralizedCoordinates(model));

    // Pelvis
    ranges = model.segment(0).ranges();
    EXPECT_EQ(ranges[0].min(), -10);
    EXPECT_EQ(ranges[0].max(), 10);
    EXPECT_EQ(ranges[1].min(), -10);
    EXPECT_EQ(ranges[1].max(), 10);
    EXPECT_EQ(ranges[2].min(), -M_PI);
    EXPECT_EQ(ranges[2].max(), M_PI);

    // BrasD
    ranges = model.segment(3).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI);
    EXPECT_EQ(ranges[0].max(), M_PI);
    EXPECT_EQ(ranges[1].min(), 0);
    EXPECT_EQ(ranges[1].max(), M_PI);

    // BrasG
    ranges = model.segment(4).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI);
    EXPECT_EQ(ranges[0].max(), M_PI);
    EXPECT_EQ(ranges[1].min(), 0);
    EXPECT_EQ(ranges[1].max(), M_PI);

    // CuisseD
    ranges = model.segment(5).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI/12);
    EXPECT_EQ(ranges[0].max(), M_PI/2+M_PI/3);

    // JambeD
    ranges = model.segment(6).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI/2-M_PI/6);
    EXPECT_EQ(ranges[0].max(), 0);

    // PiedD
    ranges = model.segment(7).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI/2);
    EXPECT_EQ(ranges[0].max(), M_PI/2);

    // CuisseG
    ranges = model.segment(8).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI/12);
    EXPECT_EQ(ranges[0].max(), M_PI/2+M_PI/3);

    // JambeG
    ranges = model.segment(9).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI/2-M_PI/6);
    EXPECT_EQ(ranges[0].max(), 0);

    // PiedG
    ranges = model.segment(10).ranges();
    EXPECT_EQ(ranges[0].min(), -M_PI/2);
    EXPECT_EQ(ranges[0].max(), M_PI/2);
}

static std::vector<double> QtestPyomecaman = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                             0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
static std::vector<double> QtestEqualsMarker = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3};

TEST(CoM, kinematics)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
    for (unsigned int i=0; i<model.nbQ(); ++i){
        Q[i] = QtestPyomecaman[i];
        Qdot[i] = QtestPyomecaman[i]*10;
        Qddot[i] = QtestPyomecaman[i]*100;
    }

    biorbd::utils::Vector3d expectedCom, expectedComDot, expectedComDdot;
    expectedCom
            << -0.0034679564024098523, 0.15680579877453169, 0.07808112642459612;
    expectedComDot
            << -0.05018973433722229, 1.4166208451420528, 1.4301750486035787;
    expectedComDdot
            << -0.7606169667295027, 11.508107073695976, 16.58853835505851;
    biorbd::utils::Vector3d com(model.CoM(Q));
    biorbd::utils::Vector3d comDot(model.CoMdot(Q, Qdot));
    biorbd::utils::Vector3d comDdot(model.CoMddot(Q, Qdot, Qddot));
    for (unsigned int i=0; i<3; ++i)
        EXPECT_NEAR(com[i], expectedCom[i], requiredPrecision);
    for (unsigned int i=0; i<3; ++i)
        EXPECT_NEAR(comDot[i], expectedComDot[i], requiredPrecision);
    for (unsigned int i=0; i<3; ++i)
        EXPECT_NEAR(comDdot[i], expectedComDdot[i], requiredPrecision);
}

TEST(Segment, copy)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::SegmentCharacteristics characteristics(
                10, biorbd::utils::Vector3d(0.5, 0.5, 0.5),
                RigidBodyDynamics::Math::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
    std::vector<biorbd::utils::Range> ranges(6);
    biorbd::rigidbody::Segment MasterSegment(
                model, "MasterSegment", "NoParent", "zyx", "yzx", ranges,
                characteristics, RigidBodyDynamics::Math::SpatialTransform());
    biorbd::rigidbody::Segment ShallowCopy(MasterSegment);
    biorbd::rigidbody::Segment ShallowCopyEqual = MasterSegment;
    biorbd::rigidbody::Segment DeepCopyNow(MasterSegment.DeepCopy());
    biorbd::rigidbody::Segment DeepCopyLater;
    DeepCopyLater.DeepCopy(MasterSegment);

    EXPECT_STREQ(MasterSegment.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopyEqual.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
    ShallowCopy.setParent("MyLovelyParent");
    EXPECT_STREQ(MasterSegment.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopyEqual.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
}

TEST(Mesh, copy)
{
    biorbd::rigidbody::Mesh MasterMesh;
    MasterMesh.setPath("./MyFile.bioMesh");
    biorbd::rigidbody::Mesh ShallowCopy(MasterMesh);
    biorbd::rigidbody::Mesh DeepCopyNow(MasterMesh.DeepCopy());
    biorbd::rigidbody::Mesh DeepCopyLater;
    DeepCopyLater.DeepCopy(MasterMesh);

    EXPECT_STREQ(MasterMesh.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyNow.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyLater.path().relativePath().c_str(), "./MyFile.bioMesh");
    ShallowCopy.setPath("./MyNewFile.bioMesh");
    EXPECT_STREQ(MasterMesh.path().relativePath().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().relativePath().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(DeepCopyNow.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyLater.path().relativePath().c_str(), "./MyFile.bioMesh");
}

static std::vector<Eigen::Vector3d> expectedMarkers = {
    Eigen::Vector3d(1.0126678074548392, 0.46575286691125295, -0.082379586527044829),
    Eigen::Vector3d(-0.18232123669751762,  0.98685937986570527,  0.46575286691125295),
    Eigen::Vector3d(0.39552020666133958, -0.18232123669751762,   1.0126678074548392),
    Eigen::Vector3d(1.0258667774186612, 1.0702910100794407, 1.1960410878390473)
};
TEST(Markers, allPositions)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    EXPECT_EQ(model.nbQ(), 6);
    EXPECT_EQ(model.nbMarkers(), 4);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i)
        Q[i] = QtestEqualsMarker[i];

    // All markers at once
    std::vector<biorbd::rigidbody::NodeSegment> markers(model.markers(Q, true, true));
    for (unsigned int i=0; i<model.nbMarkers(); ++i)
        for (unsigned int j=0; j<3; ++j)
            EXPECT_NEAR(markers[i][j], expectedMarkers[i][j], requiredPrecision);
}
TEST(Markers, individualPositions)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    EXPECT_EQ(model.nbQ(), 6);
    EXPECT_EQ(model.nbMarkers(), 4);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i)
        Q[i] = QtestEqualsMarker[i];

    // One marker at a time, only update Q once
    for (unsigned int i=0; i<model.nbMarkers(); ++i){
        biorbd::rigidbody::NodeSegment marker;
        if (i==0)
            marker = model.marker(Q, i, true, true);
        else
            marker = model.marker(Q, i, true, false);
        for (unsigned int j=0; j<3; ++j)
            EXPECT_NEAR(marker[j], expectedMarkers[i][j], requiredPrecision);
    }

    // Change Q
    Q << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1;
    std::vector<Eigen::Vector3d> expectedMarkers2 = {
        Eigen::Vector3d(1.290033288920621, 0.40925158443563553, 0.21112830525233722),
        Eigen::Vector3d(0.20066533460246938,  1.2890382781008347, 0.40925158443563558),
        Eigen::Vector3d(0.39983341664682814, 0.20066533460246938,  1.290033288920621),
        Eigen::Vector3d(1.2905320401699185, 1.2989551971389397, 1.310413178608594)
    };
    // One marker at a time, only update Q once
    for (unsigned int i=0; i<model.nbMarkers(); ++i){
        biorbd::rigidbody::NodeSegment marker;
        if (i==0)
            marker = model.marker(Q, i, true, true);
        else
            marker = model.marker(Q, i, true, false);
        for (unsigned int j=0; j<3; ++j)
            EXPECT_NEAR(marker[j], expectedMarkers2[i][j], requiredPrecision);
    }
}

TEST(Mesh, position)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int q=0; q<model.nbQ(); ++q){
        Q.setZero();
        Q[q] = 1;
        std::vector<std::vector<biorbd::utils::Vector3d>> mesh(model.meshPoints(Q));
        std::vector<biorbd::rigidbody::NodeSegment> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx)
            for (unsigned int xyz =0; xyz<3; ++xyz)
                EXPECT_NEAR(mesh[0][idx][xyz], markers[idx][xyz], requiredPrecision);
    }
    {
        Q.setOnes();
        std::vector<std::vector<biorbd::utils::Vector3d>> mesh(model.meshPoints(Q));
        std::vector<biorbd::rigidbody::NodeSegment> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx)
            for (unsigned int xyz =0; xyz<3; ++xyz)
                EXPECT_NEAR(mesh[0][idx][xyz], markers[idx][xyz], requiredPrecision);
    }
}

TEST(Dynamics, Forward)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    biorbd::rigidbody::GeneralizedAcceleration QDDot(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    QDDot_expected << 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Tau.setZero();

    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot[i], QDDot_expected[i], requiredPrecision);
}

TEST(Dynamics, ForwardLoopConstraint){
    biorbd::Model model(modelPathForLoopConstraintTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    biorbd::rigidbody::GeneralizedAcceleration QDDot_constrained(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Q.setZero();
    QDot.setZero();
    QDDot_constrained.setZero();
    QDDot_expected << 0.35935119225397999,  -10.110263945851218,  0, -27.780105329642453, 31.783042765424835,
            44.636461505611521, 0.67809701484785911, 0, 0, 15.822263679783546, 0, 0;
    Tau.setZero();

    biorbd::rigidbody::Contacts& cs(model.getConstraints());
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(model, Q, QDot, Tau, cs, QDDot_constrained);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot_constrained[i], QDDot_expected[i], requiredPrecision);
}

// TODO: confirm these tests
TEST(Dynamics, ForwardAccelerationConstraint){
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    biorbd::rigidbody::GeneralizedAcceleration QDDot_constrained(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Eigen::VectorXd forces_expected(model.nbContacts());
    Q.setOnes()/10;
    QDot.setOnes()/10;
    QDDot_expected << 1.9402069774422919,  -9.1992692111538243,  2.9930159570454702,
            5.2738378853554133, 8.9387539396273699, 6.0938738229550751, 9.9560407885164217,
            38.6297746304162, -52.159023390563554, 36.702385054876714, 38.629774630416208, -52.159023390563561,
            36.70238505487675;
    Tau.setOnes()/10;
    forces_expected << -16.344680827308579, -30.485214214095951, 112.8234134576031, -16.344680827308611,
            -30.485214214095965, 112.82341345760311;

    biorbd::rigidbody::Contacts& cs(model.getConstraints());
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(model, Q, QDot, Tau, cs, QDDot_constrained);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot_constrained[i], QDDot_expected[i], requiredPrecision);

    for (unsigned int i=0; i<cs.force.size(); ++i)
        EXPECT_NEAR(cs.force[i], forces_expected[i], requiredPrecision);
}

TEST(Kinematics, computeQdot)
{
    biorbd::Model m("models/simple_quat.bioMod");
    biorbd::rigidbody::GeneralizedVelocity QDot(m), QDot_quat_expected(m.nbQ());
    QDot << 1,2,3;
    {
        biorbd::rigidbody::GeneralizedCoordinates Q_quat(m.nbQ());
        Q_quat << 0, 0, 0, 1;
        QDot_quat_expected << 0.5, 1, 1.5, 0;

        biorbd::rigidbody::GeneralizedCoordinates QDot_quat(
                    m.computeQdot(Q_quat, QDot));
        for (unsigned int i=0; i<m.nbQ(); ++i) {
            EXPECT_NEAR(QDot_quat[i],QDot_quat_expected[i], requiredPrecision);
        }
    }
    {
        double w(0.07035975447302918);
        double x(0.7035975447302919);
        double y(0.7035975447302919);
        double z(0.07035975447302918);
        biorbd::rigidbody::GeneralizedCoordinates Q_quat(m.nbQ());
        Q_quat << x, y, z, w;
        QDot_quat_expected << 1.0202164398589233, -0.9498566853858941,
                0.45733840407468973,-1.1609359488049815;
        biorbd::rigidbody::GeneralizedCoordinates QDot_quat(
                    m.computeQdot(Q_quat,QDot));

        for (unsigned int i=0; i<m.nbQ(); ++i)
            EXPECT_NEAR(QDot_quat[i], QDot_quat_expected[i], requiredPrecision);
    }
}

#ifndef SKIP_KALMAN
#ifndef SKIP_LONG_TESTS
TEST(Kalman, markers)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::KalmanReconsMarkers kalman(model);

    // Compute reference
    biorbd::rigidbody::GeneralizedCoordinates Qref(model);
    Qref = Qref.setOnes()*0.2;
    std::vector<biorbd::rigidbody::NodeSegment> targetMarkers(model.markers(Qref));

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
    kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

    // Compare results (since the initialization of the filter is done 50X, it is expected to have converged)
    for (unsigned int i=0; i<model.nbQ(); ++i){
        EXPECT_NEAR(Q[i], Qref[i], 1e-6);
        EXPECT_NEAR(Qdot[i], 0, 1e-6);
        EXPECT_NEAR(Qddot[i], 0, 1e-6);
    }

    Qref = Qref.setOnes()*0.3;
    targetMarkers = model.markers(Qref);
    kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

    // Compare results (Here the filter should not have the time to converge)
    for (unsigned int i=0; i<model.nbQ(); ++i){
        EXPECT_GT(abs(Q[i] - Qref[i]), 1e-4);
        EXPECT_GT(abs(Qdot[i]), 5);
        EXPECT_GT(abs(Qddot[i]), 100);
    }

    // Force the filter to converge
    for (unsigned int i=0; i<100; ++i)
        kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

    // Now it should be more or less equal
    for (unsigned int i=0; i<model.nbQ(); ++i){
        EXPECT_NEAR(Q[i], Qref[i], 1e-6);
        EXPECT_NEAR(Qdot[i], 0, 1e-6);
        EXPECT_NEAR(Qddot[i], 0, 1e-6);
    }
}
#endif

#ifndef SKIP_LONG_TESTS
TEST(Kalman, imu)
{
    biorbd::Model model(modelPathForImuTesting);
    biorbd::rigidbody::KalmanReconsIMU kalman(model);

    // Compute reference
    biorbd::rigidbody::GeneralizedCoordinates Qref(model);
    Qref = Qref.setOnes()*0.2;
    std::vector<biorbd::rigidbody::IMU> targetImus(model.IMU(Qref));

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
    kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);

    // Compare results (since the initialization of the filter is done 50X, it is expected to have converged)
    for (unsigned int i=0; i<model.nbQ(); ++i){
        if (i < 2){
            // Translations are not reconstructed from IMU
            EXPECT_EQ(Q[i], 0);
            EXPECT_EQ(Qdot[i], 0);
            EXPECT_EQ(Qddot[i], 0);
        }
        else {
            EXPECT_NEAR(Q[i], Qref[i], 1e-6);
            EXPECT_NEAR(Qdot[i], 0, 1e-6);
            EXPECT_NEAR(Qddot[i], 0, 1e-6);
        }
    }

    Qref = Qref.setOnes()*0.3;
    targetImus = model.IMU(Qref);
    kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);

    // Compare results (Here the filter should not have the time to converge)
    for (unsigned int i=0; i<model.nbQ(); ++i){
        if (i<2){
            EXPECT_EQ(Q[i], 0);
            EXPECT_EQ(Qdot[i], 0);
            EXPECT_EQ(Qddot[i], 0);
        } else {
            EXPECT_GT(abs(Q[i] - Qref[i]), 1e-4);
            EXPECT_GT(abs(Qdot[i]), 1e-2);
            EXPECT_GT(abs(Qddot[i]), 1e-1);
        }
    }

    // Force the filter to converge
    for (unsigned int i=0; i<1000; ++i)
        kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);

    // Now it should be more or less equal
    for (unsigned int i=0; i<model.nbQ(); ++i){
        if (i < 2){
            // Translations are not reconstructed from IMU
            EXPECT_EQ(Q[i], 0);
            EXPECT_EQ(Qdot[i], 0);
            EXPECT_EQ(Qddot[i], 0);
        }
        else {
            EXPECT_NEAR(Q[i], Qref[i], 1e-6);
            EXPECT_NEAR(Qdot[i], 0, 1e-6);
            EXPECT_NEAR(Qddot[i], 0, 1e-6);
        }
    }
}
#endif
#endif
