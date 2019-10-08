#include <iostream>
#include <gtest/gtest.h>

#include "biorbd_c.h"
#include "Utils/String.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/NodeBone.h"
#include "RigidBody/Bone.h"
#include "RigidBody/IMU.h"
#ifndef SKIP_KALMAN
#include "RigidBody/KalmanReconsIMU.h"
#endif

static double requiredPrecision(1e-10);

static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
static std::string modelPathForIMUTesting("models/pyomecaman_withIMUs.bioMod");

TEST(BinderC, OpenCloseModel)
{
    biorbd::Model* model(c_biorbdModel(modelPathForGeneralTesting.c_str()));
    c_deleteBiorbdModel(model);
}

TEST(BinderC, nDofs)
{
    biorbd::Model* model(c_biorbdModel(modelPathForGeneralTesting.c_str()));
    EXPECT_EQ(c_nQ(model), model->nbQ());
    EXPECT_EQ(c_nQDot(model), model->nbQdot());
    EXPECT_EQ(c_nQDDot(model), model->nbQddot());
    EXPECT_EQ(c_nGeneralizedTorque(model), model->nbGeneralizedTorque());
    c_deleteBiorbdModel(model);
}

TEST(BinderC, markers)
{
    biorbd::Model* model(c_biorbdModel(modelPathForGeneralTesting.c_str()));
    EXPECT_EQ(c_nMarkers(model), model->nMarkers());

    // Markers in global reference frame
    biorbd::rigidbody::GeneralizedCoordinates Q(*model);
    double *dQ = new double[model->nbQ()];
    for (unsigned int i=0; i<model->nbQ(); ++i){
        Q[i] = 0.1;
        dQ[i] = 0.1;
    }
    double *dMarkersInGlobal = new double[model->nMarkers()*3];
    c_markers(model, dQ, dMarkersInGlobal);
    std::vector<biorbd::rigidbody::NodeBone> markersInGlobal(model->markers(Q));
    for (unsigned int i=0; i<model->nMarkers(); ++i)
        for (unsigned int j=0; j<3; ++j)
            EXPECT_NEAR(dMarkersInGlobal[i*3+j], markersInGlobal[i][j], requiredPrecision);
    delete[] dMarkersInGlobal;

    // Add a marker
    double newMarkerPosition[3] = {1, 2, 3};
    unsigned int nMarkersBeforeAdding(model->nMarkers());
    c_addMarker(model, newMarkerPosition, "MyNewMarker", model->bone(1).name().c_str(), false, true, "x");
    EXPECT_EQ(model->nMarkers(), nMarkersBeforeAdding + 1);
    biorbd::rigidbody::NodeBone newMarker(model->marker(nMarkersBeforeAdding));
    EXPECT_STREQ(newMarker.name().c_str(), "MyNewMarker");
    EXPECT_STREQ(newMarker.parent().c_str(), model->bone(1).name().c_str());
    EXPECT_EQ(newMarker.isTechnical(), false);
    EXPECT_EQ(newMarker.isAnatomical(), true);
    EXPECT_STREQ(newMarker.axesToRemove().c_str(), "x");

    // Markers in local reference frame
    double *dMarkersInLocal = new double[model->nMarkers()*3];
    c_markersInLocal(model, dMarkersInLocal);
    std::vector<biorbd::rigidbody::NodeBone> markersInLocal(model->markers());
    for (unsigned int i=0; i<model->nMarkers(); ++i)
        for (unsigned int j=0; j<3; ++j)
            EXPECT_NEAR(dMarkersInLocal[i*3+j], markersInLocal[i][j], requiredPrecision);
    delete[] dMarkersInLocal;

    delete[] dQ;
    c_deleteBiorbdModel(model);
}

TEST(BinderC, jcs)
{
    biorbd::Model* model(c_biorbdModel(modelPathForGeneralTesting.c_str()));

    // Get angle sequence of some bodies
    for (unsigned int i=0; i<model->nbBone(); ++i){
        char sequence[4];
        c_boneRotationSequence(model, model->bone(i).name().c_str(), sequence);
        EXPECT_STREQ(sequence, model->bone(i).seqR().c_str());
    }

    // JCS in global reference frame
    biorbd::rigidbody::GeneralizedCoordinates Q(*model);
    double *dQ = new double[model->nbQ()];
    for (unsigned int i=0; i<model->nbQ(); ++i){
        Q[i] = 0.1;
        dQ[i] = 0.1;
    }
    double *dRtInGlobal = new double[model->nbBone()*4*4];
    c_globalJCS(model, dQ, dRtInGlobal);
    std::vector<biorbd::utils::RotoTrans> jcsInGlobal(model->allGlobalJCS(Q));
    for (unsigned int i=0; i<model->nbBone(); ++i)
        for (unsigned int row=0; row<4; ++row)
            for (unsigned int col=0; col<4; ++col)
                EXPECT_NEAR(dRtInGlobal[i*16+col*4+row], jcsInGlobal[i](row, col), requiredPrecision);
    delete[] dRtInGlobal;

    // JCS in local reference frame
    double *dRtInLocal = new double[model->nbBone()*4*4];
    c_globalJCS(model, dQ, dRtInLocal);
    std::vector<biorbd::utils::RotoTrans> jcsInLocal(model->allGlobalJCS());
    for (unsigned int i=0; i<model->nbBone(); ++i)
        for (unsigned int row=0; row<4; ++row)
            for (unsigned int col=0; col<4; ++col)
                EXPECT_NEAR(dRtInLocal[i*16+col*4+row], jcsInLocal[i](row, col), requiredPrecision);
    delete[] dRtInLocal;

    delete[] dQ;
    c_deleteBiorbdModel(model);
}

TEST(BinderC, imu)
{
    biorbd::Model* model(c_biorbdModel(modelPathForIMUTesting.c_str()));
    EXPECT_EQ(c_nIMUs(model), model->nIMUs());

    biorbd::utils::RotoTrans RT;
    RT.transformCardanToMatrix(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1), "xyz");
    double rt[16];
    for (unsigned int i=0; i<4; ++i)
        for (unsigned int j=0; j<4; ++j)
            rt[j*4 + i] = RT(i, j);

    // Add an imu
    unsigned int nImuBeforeAdding(model->nIMUs());
    c_addIMU(model, rt, "MyNewIMU", "Tete", false, true);
    EXPECT_EQ(model->nIMUs(), nImuBeforeAdding+1);
    biorbd::rigidbody::IMU newImu(model->IMU(nImuBeforeAdding));
    EXPECT_STREQ(newImu.name().c_str(), "MyNewIMU");
    EXPECT_STREQ(newImu.parent().c_str(), "Tete");
    EXPECT_EQ(newImu.isTechnical(), false);
    EXPECT_EQ(newImu.isAnatomical(), true);

    c_deleteBiorbdModel(model);
}

#ifndef SKIP_KALMAN
TEST(BinderC, kalmanImu)
{
    biorbd::Model* model(c_biorbdModel(modelPathForIMUTesting.c_str()));
    biorbd::rigidbody::KalmanReconsIMU kalman(*model);
    biorbd::rigidbody::KalmanReconsIMU* kalman_c(c_BiorbdKalmanReconsIMU(model));

    // Compute reference
    std::vector<biorbd::rigidbody::IMU> targetImus;
    for (unsigned int i=0; i<model->nIMUs(); ++i){
        biorbd::utils::RotoTrans rt;
        rt.transformCardanToMatrix(Eigen::Vector3d(0.1*i, 0.1*i, 0.1*i), Eigen::Vector3d(0.1*i, 0.1*i, 0.1*i), "xyz");
        targetImus.push_back(rt);
    }
    biorbd::rigidbody::GeneralizedCoordinates Q(*model), Qdot(*model), Qddot(*model);
    kalman.reconstructFrame(*model, targetImus, &Q, &Qdot, &Qddot);

    // Compute from C-interface
    double* target = new double[model->nIMUs()*3*3];
    for (unsigned int i=0; i<model->nIMUs(); ++i)
        for (unsigned int row=0; row<3; ++row)
            for (unsigned int col=0; col<3; ++col)
                target[i*9+3*col+row] = targetImus[i](row, col);
    double* Q_c = new double[model->nbQ()], *Qdot_c = new double[model->nbQ()], *Qddot_c = new double[model->nbQ()];
    c_BiorbdKalmanReconsIMUstep(model, kalman_c, target, Q_c, Qdot_c, Qddot_c);

    // Compare results
    for (unsigned int i=0; i<model->nbQ(); ++i){
        EXPECT_DOUBLE_EQ(Q_c[i], Q[i]);
        EXPECT_DOUBLE_EQ(Qdot_c[i], Qdot[i]);
        EXPECT_DOUBLE_EQ(Qddot_c[i], Qddot[i]);
    }

    delete[] target;
    delete[] Q_c;
    delete[] Qdot_c;
    delete[] Qddot_c;
    c_deleteBiorbdKalmanReconsIMU(kalman_c);
    c_deleteBiorbdModel(model);
}
#endif

TEST(BinderC, math)
{
    // Simple matrix multiplaction (RT3 = RT1 * RT2)
    {
        biorbd::utils::RotoTrans RT1, RT2, RT3;
        RT1.transformCardanToMatrix(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1), "xyz");
        RT2.transformCardanToMatrix(Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(0.2, 0.2, 0.2), "xyz");
        RT3 = RT1 * RT2;

        double rt1[16], rt2[16], rt3[16];
        for (unsigned int i=0; i<4; ++i)
            for (unsigned int j=0; j<4; ++j){
                rt1[j*4 + i] = RT1(i, j);
                rt2[j*4 + i] = RT2(i, j);
            }
        c_matrixMultiplication(rt1, rt2, rt3);

        for (unsigned int row=0; row<4; ++row)
            for (unsigned int col=0; col<4; ++col)
                EXPECT_NEAR(rt3[col*4+row], RT3(row, col), requiredPrecision);
    }

    // Mean multiple matrices
    {
        std::vector<biorbd::utils::RotoTrans> allRT(3);
        biorbd::utils::RotoTrans meanRT;
        allRT[0].transformCardanToMatrix(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1), "xyz");
        allRT[1].transformCardanToMatrix(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.2, 0.2, 0.2), "xyz");
        allRT[2].transformCardanToMatrix(Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(0.3, 0.3, 0.3), "xyz");
        meanRT = biorbd::utils::RotoTrans::mean(allRT);

        double* rt = new double[meanRT.size()*16];
        double mean_rt[16];
        for (unsigned int i=0; i<meanRT.size(); ++i)
            for (unsigned int col=0; col<4; ++col)
                for (unsigned int row=0; row<4; ++row){
                    rt[i*16 + col*4 + row] = allRT[i](row, col);
                }
        c_meanRT(rt, static_cast<unsigned int>(allRT.size()), mean_rt);
        delete[] rt;

        for (unsigned int row=0; row<4; ++row)
            for (unsigned int col=0; col<4; ++col)
                EXPECT_NEAR(mean_rt[col*4+row], meanRT(row, col), requiredPrecision);
    }

    // Project jcs onto another (RT3 = RT1.tranpose() * RT2)
    {
        biorbd::utils::RotoTrans RT1, RT2, RT3;
        RT1.transformCardanToMatrix(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1), "xyz");
        RT2.transformCardanToMatrix(Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(0.2, 0.2, 0.2), "xyz");
        RT3 = RT1.transpose() * RT2;

        double rt1[16], rt2[16], rt3[16];
        for (unsigned int i=0; i<4; ++i)
            for (unsigned int j=0; j<4; ++j){
                rt1[j*4 + i] = RT1(i, j);
                rt2[j*4 + i] = RT2(i, j);
            }
        c_projectJCSinParentBaseCoordinate(rt1, rt2, rt3);

        for (unsigned int row=0; row<4; ++row)
            for (unsigned int col=0; col<4; ++col)
                EXPECT_NEAR(rt3[col*4+row], RT3(row, col), requiredPrecision);
    }

    // Get the cardan angles from a matrix
    {
        biorbd::utils::RotoTrans RT;
        RT.transformCardanToMatrix(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1), "xyz");
        biorbd::utils::Vector realCardan(biorbd::utils::RotoTrans::transformMatrixToCardan(RT, "xyz"));

        double rt[16];
        for (unsigned int i=0; i<4; ++i)
            for (unsigned int j=0; j<4; ++j)
                rt[j*4 + i] = RT(i, j);

        double cardan[3];
        c_transformMatrixToCardan(rt, "xyz", cardan);

        for (unsigned int i=0; i<3; ++i)
            EXPECT_NEAR(cardan[i], realCardan[i], requiredPrecision);
    }
}
