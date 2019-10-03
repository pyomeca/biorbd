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
#include "RigidBody/NodeBone.h"
#include "RigidBody/Bone.h"
#include "RigidBody/IMU.h"
#ifndef SKIP_KALMAN
#include "RigidBody/KalmanReconsMarkers.h"
#include "RigidBody/KalmanReconsIMU.h"
#endif


static double requiredPrecision(1e-10);

#ifdef MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
static std::string modelPathForImuTesting("models/pyomecaman_withIMUs.bioMod");
#else // MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
static std::string modelPathMeshEqualsMarker("models/meshsEqualMarkers.bioMod");
static std::string modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");

TEST(Bone, copy)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::BoneCaracteristics caract(10, biorbd::utils::Node3d(0.5, 0.5, 0.5), RigidBodyDynamics::Math::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
    biorbd::rigidbody::Bone MasterBone(model, "MasterBone", "NoParent", "zyx", "yzx", caract, RigidBodyDynamics::Math::SpatialTransform());
    biorbd::rigidbody::Bone ShallowCopy(MasterBone);
    biorbd::rigidbody::Bone DeepCopyNow(MasterBone.DeepCopy());
    biorbd::rigidbody::Bone DeepCopyLater;
    DeepCopyLater.DeepCopy(MasterBone);

    EXPECT_STREQ(MasterBone.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
    ShallowCopy.setParent("MyLovelyParent");
    EXPECT_STREQ(MasterBone.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
}

TEST(BoneMesh, copy)
{
    biorbd::rigidbody::BoneMesh MasterMesh;
    MasterMesh.setPath("./MyFile.bioMesh");
    biorbd::rigidbody::BoneMesh ShallowCopy(MasterMesh);
    biorbd::rigidbody::BoneMesh DeepCopyNow(MasterMesh.DeepCopy());
    biorbd::rigidbody::BoneMesh DeepCopyLater;
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

static std::vector<double> QforMarkers = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3};
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
    EXPECT_EQ(model.nMarkers(), 4);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i)
        Q[i] = QforMarkers[i];

    // All markers at once
    std::vector<biorbd::rigidbody::NodeBone> markers(model.markers(Q, true, true));
    for (unsigned int i=0; i<model.nMarkers(); ++i)
        for (unsigned int j=0; j<3; ++j)
            EXPECT_NEAR(markers[i][j], expectedMarkers[i][j], requiredPrecision);
}
TEST(Markers, individualPositions)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    EXPECT_EQ(model.nbQ(), 6);
    EXPECT_EQ(model.nMarkers(), 4);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i)
        Q[i] = QforMarkers[i];

    // One marker at a time, only update Q once
    for (unsigned int i=0; i<model.nMarkers(); ++i){
        biorbd::rigidbody::NodeBone marker;
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
    for (unsigned int i=0; i<model.nMarkers(); ++i){
        biorbd::rigidbody::NodeBone marker;
        if (i==0)
            marker = model.marker(Q, i, true, true);
        else
            marker = model.marker(Q, i, true, false);
        for (unsigned int j=0; j<3; ++j)
            EXPECT_NEAR(marker[j], expectedMarkers2[i][j], requiredPrecision);
    }
}

TEST(BoneMesh, position)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int q=0; q<model.nbQ(); ++q){
        Q.setZero();
        Q[q] = 1;
        std::vector<std::vector<biorbd::utils::Node3d>> mesh(model.meshPoints(Q));
        std::vector<biorbd::rigidbody::NodeBone> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx)
            for (unsigned int xyz =0; xyz<3; ++xyz)
                EXPECT_NEAR(mesh[0][idx][xyz], markers[idx][xyz], requiredPrecision);
    }
    {
        Q.setOnes();
        std::vector<std::vector<biorbd::utils::Node3d>> mesh(model.meshPoints(Q));
        std::vector<biorbd::rigidbody::NodeBone> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx)
            for (unsigned int xyz =0; xyz<3; ++xyz)
                EXPECT_NEAR(mesh[0][idx][xyz], markers[idx][xyz], requiredPrecision);
    }
}

TEST(Dynamics, Forward)
{
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
        EXPECT_NEAR(QDDot[i], QDDot_expected[i], requiredPrecision);
}

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

    biorbd::rigidbody::Contacts& cs(model.getConstraints());
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(model, Q, QDot, Tau, cs, QDDot_constrained);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_NEAR(QDDot_constrained[i], QDDot_expected[i], requiredPrecision);
}

// TODO: confirm these tests
TEST(Dynamics, ForwardAccelerationConstraint){
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::GeneralizedCoordinates Q(model), QDot(model), QDDot_constrained(model), QDDot_expected(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Eigen::VectorXd forces_expected(model.nContacts());
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

#ifndef SKIP_KALMAN
#ifndef SKIP_LONG_TESTS
TEST(Kalman, markers)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::KalmanReconsMarkers kalman(model);

    // Compute reference
    biorbd::rigidbody::GeneralizedCoordinates Qref(model);
    Qref = Qref.setOnes()*0.2;
    std::vector<biorbd::rigidbody::NodeBone> targetMarkers(model.markers(Qref));

    biorbd::rigidbody::GeneralizedCoordinates Q(model), Qdot(model), Qddot(model);
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

    biorbd::rigidbody::GeneralizedCoordinates Q(model), Qdot(model), Qddot(model);
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
