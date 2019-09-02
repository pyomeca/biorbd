#define BIORBD_API_EXPORTS
#include "RigidBody/KalmanReconsIMU.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/IMU.h"

biorbd::rigidbody::KalmanReconsIMU::KalmanReconsIMU() :
    biorbd::rigidbody::KalmanRecons(),
    m_PpInitial(std::make_shared<biorbd::utils::Matrix>()),
    m_firstIteration(std::make_shared<bool>(true))
{

}

biorbd::rigidbody::KalmanReconsIMU::KalmanReconsIMU(
        biorbd::Model &model,
        biorbd::rigidbody::KalmanRecons::KalmanParam params) :
    biorbd::rigidbody::KalmanRecons(model, model.nTechIMUs()*9, params),
    m_PpInitial(std::make_shared<biorbd::utils::Matrix>()),
    m_firstIteration(std::make_shared<bool>(true))
{
    // Initialiser le filtre
    initialize();
}

biorbd::rigidbody::KalmanReconsIMU biorbd::rigidbody::KalmanReconsIMU::DeepCopy() const
{
    biorbd::rigidbody::KalmanReconsIMU copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::KalmanReconsIMU::DeepCopy(const biorbd::rigidbody::KalmanReconsIMU &other)
{
    biorbd::rigidbody::KalmanRecons::DeepCopy(other);
    *m_PpInitial = other.m_PpInitial->DeepCopy();
    *m_firstIteration = *other.m_firstIteration;
}

void biorbd::rigidbody::KalmanReconsIMU::initialize()
{
    biorbd::rigidbody::KalmanRecons::initialize();

    // Se souvenir de m_Pp de départ
    *m_PpInitial = *m_Pp;
}

void biorbd::rigidbody::KalmanReconsIMU::manageOcclusionDuringIteration(
        biorbd::utils::Matrix &InvTp,
        biorbd::utils::Vector &measure,
        const std::vector<unsigned int> &occlusion)
{
    for (unsigned int i = 0; i < occlusion.size(); ++i)
         for (unsigned int j=occlusion[i] * 9; j< occlusion[i] * 9+9; ++j){
             InvTp(j,j) = 0; // Artéfact du au fait que m_R a une valeur à (j:j+2,j:j+2)
             measure[j] = 0;
         }
}

bool biorbd::rigidbody::KalmanReconsIMU::first()
{
    return *m_firstIteration;
}

void biorbd::rigidbody::KalmanReconsIMU::reconstructFrame(
        biorbd::Model &m,
        const std::vector<biorbd::utils::RotoTrans> &IMUobs,
        biorbd::rigidbody::GeneralizedCoordinates *Q,
        biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        biorbd::rigidbody::GeneralizedCoordinates *Qddot)
{
    // Séparer les IMUobs en un grand vecteur
    biorbd::utils::Vector T(static_cast<unsigned int>(3*3*IMUobs.size())); // Matrice 3*3 * nIMU
    for (unsigned int i=0; i<IMUobs.size(); ++i)
        for (unsigned int j=0; j<3; ++j)
           T.block(9*i+3*j, 0, 3, 1) = IMUobs[i].block(0,j,3,1);

    // Reconstruire la cinématique
    reconstructFrame(m, T, Q, Qdot, Qddot);
}


void biorbd::rigidbody::KalmanReconsIMU::reconstructFrame(
        biorbd::Model &model,
        const utils::Vector &IMUobs,
        biorbd::rigidbody::GeneralizedCoordinates *Q,
        biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        biorbd::rigidbody::GeneralizedCoordinates *Qddot)
{
    // Une itération du filtre de Kalman
    if (*m_firstIteration){
        *m_firstIteration = false;
        for (unsigned int i=0; i<500; ++i){
            // La premiere fois, appeler de facon recursive pour avoir une position initiale decente
            reconstructFrame(model, IMUobs, nullptr, nullptr, nullptr);

            // Remettre Pp à initial (parce qu'on ne s'intéresse pas à la vitesse pour se rendre à la position initiale
            m_xp->block(*m_nDof, 0, *m_nDof*2, 1) = Eigen::VectorXd::Zero(*m_nDof*2); // Mettre vitesse et accélération à 0
        }
    }

    // État projeté
    const biorbd::utils::Vector& xkm(*m_A * *m_xp);
    biorbd::rigidbody::GeneralizedCoordinates Q_tp(xkm.topRows(*m_nDof));
    model.UpdateKinematicsCustom (&Q_tp, nullptr, nullptr);

    // Markers projetés
    const std::vector<biorbd::rigidbody::IMU>& zest_tp = model.technicalIMU(Q_tp, false);
    // Jacobienne
    const std::vector<biorbd::utils::Matrix>& J_tp = model.TechnicalIMUJacobian(Q_tp, false);
    // Faire une seule matrice pour zest et Jacobienne
    biorbd::utils::Matrix H(biorbd::utils::Matrix::Zero(*m_nMeasure, *m_nDof*3)); // 3*nCentrales => X,Y,Z ; 3*nDof => Q, Qdot, Qddot
    biorbd::utils::Vector zest(biorbd::utils::Vector::Zero(*m_nMeasure));
    std::vector<unsigned int> occlusionIdx;
    for (unsigned int i=0; i<*m_nMeasure/9; ++i){
        double sum = 0;
        for (unsigned int j = 0; j < 9; ++j) // Calculer la norme des 9 composantes
            sum += IMUobs(i*9+j)*IMUobs(i*9+j);
        if (sum != 0.0 && sum == sum){ // S'il y a un imu (pas de zéro ou nan)
            H.block(i*9,0,9,*m_nDof) = J_tp[i];
            const Eigen::Matrix3d& rot = zest_tp[i].rot();
            for (unsigned int j = 0; j < 3; ++j)
                zest.block(i*9+j*3, 0, 3, 1) = rot.block(0, j, 3, 1);
        }
        else
            occlusionIdx.push_back(i);
    }

    // Faire le filtre
    iteration(IMUobs, zest, H, occlusionIdx);

    getState(Q, Qdot, Qddot);
}

void biorbd::rigidbody::KalmanReconsIMU::reconstructFrame()
{
    biorbd::utils::Error::error(false, "Implémentation impossible");
}
