#include <vector>

#include "biorbd_c.h"

#include "ModelReader.h"
#include "ModelWriter.h"
#include "Utils/String.h"
#include "Utils/RotoTrans.h"
#include "Utils/RotoTransNode.h"
#include "RigidBody/Bone.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/NodeBone.h"
#ifndef SKIP_KALMAN
#include "RigidBody/KalmanReconsIMU.h"
#endif

biorbd::Model* c_biorbdModel(
        const char* pathToModel) {
    return new biorbd::Model(
                biorbd::Reader::readModelFile(
                    biorbd::utils::String(pathToModel)));
}
void c_deleteBiorbdModel(
        biorbd::Model* model) {
    delete model;
}
void c_writeBiorbdModel(
        biorbd::Model* model,
        const char * path) {
    biorbd::Writer::writeModel(*model, biorbd::utils::Path(path));
}



// Joints functions
void c_boneRotationSequence(
        biorbd::Model* m,
        const char* segName,
        char* seq) {
    // Memory for seq must be already allocated
    biorbd::utils::String sequence(m->bone(segName).seqR());
    snprintf(seq, sequence.length(), "%s", sequence.c_str());
}
void c_localJCS(
        biorbd::Model* m,
        int i,
        double* rt_out) {
    biorbd::utils::RotoTrans RT(
                m->bone(static_cast<unsigned int>(i)).localJCS());
    dispatchRToutput(RT, rt_out);
}
void c_globalJCS(
        biorbd::Model* m,
        const double* Q,
        double* jcs) {
    // Dispatch des données d'entrée
    biorbd::rigidbody::GeneralizedCoordinates eQ(dispatchQinput(m, Q));

    // Récupérer JCS
    std::vector<biorbd::utils::RotoTrans> pre_jcs = m->allGlobalJCS(eQ);

    // Dispatch de l'output
    dispatchRToutput(pre_jcs, jcs);
}




// dof functions
int c_nQ(
        biorbd::Model* model) {
    return static_cast<int>(model->nbQ());
}
int c_nQDot(
        biorbd::Model* model) {
    return static_cast<int>(model->nbQdot());
}
int c_nQDDot(
        biorbd::Model* model) {
    return static_cast<int>(model->nbQddot());
}
int c_nGeneralizedTorque(
        biorbd::Model* model) {
    return static_cast<int>(model->nbGeneralizedTorque());
}


// Markers functions
int c_nMarkers(
        biorbd::Model* model) {
    return static_cast<int>(model->nMarkers());
}
void c_markersInLocal(
        biorbd::Model* model,
        double* markPos) {
    // Prepare output
    dispatchMarkersOutput(model->markers(), markPos);
}
void c_markers(
        biorbd::Model* model,
        const double* Q,
        double* markPos,
        bool removeAxis,
        bool updateKin) {
    // Prepare parameters
    biorbd::rigidbody::GeneralizedCoordinates eQ(dispatchQinput(model, Q));

    // Call the main function
    std::vector<biorbd::rigidbody::NodeBone> pos(
                model->markers(eQ, removeAxis, updateKin));

    // Prepare output
    dispatchMarkersOutput(pos, markPos);
}
void c_addMarker(
        biorbd::Model *model,
        const double *markPos,
        const char* name,
        const char* parentName,
        bool technical,
        bool anatomical,
        const char* axesToRemove) {
    int parent_int = static_cast<int>(model->GetBodyId(parentName));
    biorbd::utils::Node3d pos(dispatchMarkersInput(markPos));
    model->addMarker(pos, name, parentName, technical,
                     anatomical, axesToRemove, parent_int);
}


// IMUs functions
int c_nIMUs(
        biorbd::Model* model) {
    return static_cast<int>(model->nIMUs());
}
void c_addIMU(
        biorbd::Model *model,
        const double *imuRT,
        const char *name,
        const char *parentName,
        bool technical,
        bool anatomical) {
    biorbd::utils::RotoTransNode pos(dispatchRTinput(imuRT), name, parentName);
    model->addIMU(pos, technical, anatomical);
}


// Kalman IMU
#ifndef SKIP_KALMAN
biorbd::rigidbody::KalmanReconsIMU* c_BiorbdKalmanReconsIMU(
        biorbd::Model* model,
        double* QinitialGuess,
        double freq,
        double noiseF,
        double errorF) {
    // Créer un pointeur sur un filtre de kalman
    biorbd::rigidbody::KalmanReconsIMU* kalman =
            new biorbd::rigidbody::KalmanReconsIMU(
                *model, biorbd::rigidbody::KalmanReconsIMU::KalmanParam(
                    freq, noiseF, errorF));

    // Mettre le initial guess
    biorbd::rigidbody::GeneralizedCoordinates e_QinitialGuess(*model);
    if (QinitialGuess != nullptr) {
        for (size_t i = 0; i < model->nbQ(); ++i) {
            e_QinitialGuess[static_cast<int>(i)] = QinitialGuess[i];
        }
        kalman->setInitState(&e_QinitialGuess);
    }

    return kalman;
}
void c_deleteBiorbdKalmanReconsIMU(biorbd::rigidbody::KalmanReconsIMU* model) {
    delete model;
}
void c_BiorbdKalmanReconsIMUstep(
        biorbd::Model* model,
        biorbd::rigidbody::KalmanReconsIMU* kalman,
        double* imu,
        double* Q,
        double* QDot,
        double* QDDot) {
    // Copier les valeurs des matrices de rotation des IMUs dans un stl Vector
    Eigen::VectorXd T(3*3*model->nIMUs());  // Matrice 3*3 * nIMU
    for (unsigned int i = 0; i < model->nIMUs(); ++i) {
        for (unsigned int j = 0; j < 9; ++j) {  // matrice 3*3
            T[9*i+j] = imu[9*i+j];
        }
    }

    // Se faire des entrés sur Q, QDot et QDDot
    biorbd::rigidbody::GeneralizedCoordinates
            e_Q(*model), e_QDot(*model), e_QDDot(*model);

    // Faire le filtre
    kalman->reconstructFrame(*model, T, &e_Q, &e_QDot, &e_QDDot);

    // Transcrire les réponses vers les arguments de sortie
    dispatchQoutput(e_Q, Q);
    dispatchQoutput(e_QDot, QDot);
    dispatchQoutput(e_QDDot, QDDot);
}
#endif

// Math functions
void c_matrixMultiplication(
        const double* M1,
        const double* M2,
        double* Mout) {
    // Recueillir les données d'entrée
    biorbd::utils::RotoTrans mM1(dispatchRTinput(M1));
    biorbd::utils::RotoTrans mM2(dispatchRTinput(M2));

    // Projeter et préparer les données de sortie
    dispatchRToutput(mM1 * mM2, Mout);
}
void c_meanRT(
        const double *imuRT,
        unsigned int nFrame,
        double* imuRT_mean) {
    std::vector<biorbd::utils::RotoTrans> m;

    // Dispatch des données d'entrée
    for (unsigned int i = 0; i < nFrame; ++i) {
        m.push_back(dispatchRTinput(&imuRT[i*16]));
    }

    // Calcul de la moyenne
    biorbd::utils::RotoTrans mMean = biorbd::utils::RotoTrans::mean(m);

    // Dispatch des données de sortie
    dispatchRToutput(mMean, imuRT_mean);
}
void c_projectJCSinParentBaseCoordinate(
        const double* parent,
        const double* jcs,
        double * out) {
    // Recueillir les données d'entrée
    biorbd::utils::RotoTrans aParent(dispatchRTinput(parent));
    biorbd::utils::RotoTrans aJcs(dispatchRTinput(jcs));

    // Projeter et préparer les données de sortie
    dispatchRToutput(aParent.transpose() * aJcs, out);
}
void c_transformMatrixToCardan(
        const double *M,
        const char *sequence,
        double* cardanOut) {
    biorbd::utils::RotoTrans mM(dispatchRTinput(M));
    biorbd::utils::String seq(sequence);

    biorbd::utils::Vector cardan(
                biorbd::utils::RotoTrans::transformMatrixToCardan(mM, seq));

    // On assume que la mémoire pour cardanOut a déjà été octroyée
    dispatchDoubleOutput(cardan, cardanOut);
}


// Fonctions de dispatch des données d'entré ou de sortie
biorbd::utils::Node3d dispatchMarkersInput(
        const double * pos) {
    return biorbd::utils::Node3d(pos[0], pos[1], pos[2]);
}
void dispatchMarkersOutput(
        const std::vector<biorbd::rigidbody::NodeBone> &allMarkers,
        double* markers) {
    // Warning markers must already be allocated!
    for (size_t i = 0; i < allMarkers.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            markers[i*3+j] = allMarkers[i][static_cast<int>(j)];
        }
    }
}
biorbd::rigidbody::GeneralizedCoordinates dispatchQinput(
        biorbd::Model* model,
        const double* Q) {
    // Prepare parameters
    biorbd::rigidbody::GeneralizedCoordinates eQ(*model);
    for (int i = 0; i < static_cast<int>(model->nbQ()); ++i) {
        eQ[i] = Q[i];
    }
    return eQ;
}
void dispatchQoutput(
        const biorbd::rigidbody::GeneralizedCoordinates &eQ,
        double*Q) {
    // Warnging Q must already be allocated
    for (unsigned int i = 0; i < eQ.size(); ++i) {
        Q[i] = eQ[i];
    }
}
void dispatchDoubleOutput(
        const biorbd::utils::Vector& e,
        double* d) {
    // Warning output must already be allocated
    for (unsigned int i = 0; i < e.size(); ++i) {
        d[i] = e[i];
    }
}
biorbd::utils::RotoTrans dispatchRTinput(
        const double* rt) {
    biorbd::utils::RotoTrans pos;
    pos <<  rt[0], rt[4], rt[8], rt[12],
            rt[1], rt[5], rt[9], rt[13],
            rt[2], rt[6], rt[10], rt[14],
            rt[3], rt[7], rt[11], rt[15];
    return pos;
}
void dispatchRToutput(
        const biorbd::utils::RotoTrans& rt_in,
        double* rt_out) {
    // Attention la mémoire doit déjà être allouée pour rt_out
    for (unsigned int i = 0; i < 16; ++i) {
        rt_out[i] = rt_in(i%4, i/4);
    }
}
void dispatchRToutput(
        const std::vector<biorbd::utils::RotoTrans>& rt_in,
        double* rt_out) {
    // Attention la mémoire doit déjà être allouée pour rt_out
    for (unsigned int i = 0; i < rt_in.size(); ++i) {
        for (unsigned int j = 0; j < 16; ++j) {
            rt_out[i*16+j] = rt_in[i](j%4, j/4);
        }
    }
}








//// Spécifique à des projets (IMU sous Unity)
// void c_alignSpecificAxisWithParentVertical(
//         const double* parentRT,
//         const double * childRT,
//         int idxAxe,
//         double * rotation) {
// // Matrices à aliger (axe 2 de r1 avec axe idxAxe de r2)
// s2mAttitude r1(dispatchRTinput(parentRT));
// s2mAttitude r2(dispatchRTinput(childRT));

//    s2mAttitude rotationMat =
//            s2mIMU_Unity_Optim::alignSpecificAxisWithParentVertical(
//                r1, r2, idxAxe);

//    dispatchRToutput(rotationMat, rotation);
//}
