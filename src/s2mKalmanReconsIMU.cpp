#define BIORBD_API_EXPORTS
#include "../include/s2mKalmanReconsIMU.h"

s2mKalmanReconsIMU::s2mKalmanReconsIMU(s2mMusculoSkeletalModel &m, s2mKalmanRecons::s2mKalmanParam params) :
    s2mKalmanRecons(m, m.nTechIMUs()*9, params),
    m_firstIteration(true)
{

    // Initialiser le filtre
    initialize();

}

void s2mKalmanReconsIMU::initialize(){
    s2mKalmanRecons::initialize();

    // Se souvenir de m_Pp de départ
    m_PpInitial = m_Pp;
}

void s2mKalmanReconsIMU::manageOcclusionDuringIteration(Eigen::MatrixXd &InvTp, Eigen::VectorXd &measure, const std::vector<unsigned int> &occlusion)
{
    for (unsigned int i = 0; i < occlusion.size(); ++i)
         for (unsigned int j=occlusion[i] * 9; j< occlusion[i] * 9+9; ++j){
             InvTp(j,j) = 0; // Artéfact du au fait que m_R a une valeur à (j:j+2,j:j+2)
             measure[j] = 0;
         }
}

bool s2mKalmanReconsIMU::first()
{
    return m_firstIteration;
}

void s2mKalmanReconsIMU::reconstructFrame(s2mMusculoSkeletalModel &m, const std::vector<s2mAttitude> &IMUobs, s2mGenCoord *Q, s2mGenCoord *Qdot, s2mGenCoord *Qddot){
    // Séparer les IMUobs en un grand vecteur
    Eigen::VectorXd T(3*3*IMUobs.size()); // Matrice 3*3 * nIMU
    for (unsigned int i=0; i<IMUobs.size(); ++i)
        for (unsigned int j=0; j<3; ++j)
           T.block(9*i+3*j,0,3,1) = IMUobs[i].block(0,j,3,1);

    // Reconstruire la cinématique
    reconstructFrame(m, T, Q, Qdot, Qddot);
}


void s2mKalmanReconsIMU::reconstructFrame(s2mMusculoSkeletalModel &m, const Eigen::VectorXd &IMUobs, s2mGenCoord *Q, s2mGenCoord *Qdot, s2mGenCoord *Qddot){
    // Une itération du filtre de Kalman
    if (m_firstIteration){
        m_firstIteration = false;
        for (unsigned int i=0; i<500; ++i){
            // La premiere fois, appeler de facon recursive pour avoir une position initiale decente
            reconstructFrame(m, IMUobs, nullptr, nullptr, nullptr);

            // Remettre Pp à initial (parce qu'on ne s'intéresse pas à la vitesse pour se rendre à la position initiale
            m_xp.block(m_nDof, 0, m_nDof*2,1) = Eigen::VectorXd::Zero(m_nDof*2); // Mettre vitesse et accélération à 0
        }
    }

    // État projeté
    Eigen::VectorXd xkm = m_A * m_xp;
    Eigen::VectorXd Q_tp = xkm.topRows(m_nDof);
    RigidBodyDynamics::UpdateKinematicsCustom (m, &Q_tp, nullptr, nullptr);

    // Markers projetés
    std::vector<s2mIMU> zest_tp = m.technicalIMU(m, Q_tp, false);
    // Jacobienne
    std::vector<Eigen::MatrixXd> J_tp = m.TechnicalIMUJacobian(m, Q_tp, false);
    // Faire une seule matrice pour zest et Jacobienne
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_nMeasure, m_nDof*3); // 3*nCentrales => X,Y,Z ; 3*nDof => Q, Qdot, Qddot
    Eigen::VectorXd zest = Eigen::VectorXd::Zero(m_nMeasure);
    std::vector<unsigned int> occlusionIdx;
    for (unsigned int i=0; i<m_nMeasure/9; ++i){
        double sum = 0;
        for (unsigned int j = 0; j < 9; ++j) // Calculer la norme des 9 composantes
            sum += IMUobs(i*9+j)*IMUobs(i*9+j);
        if (sum != 0.0 && sum == sum){ // S'il y a un imu (pas de zéro ou nan)
            H.block(i*9,0,9,m_nDof) = *(J_tp.begin()+i);
            Eigen::Matrix3d rot = (*(zest_tp.begin()+i)).rot();
            for (unsigned int j = 0; j < 3; ++j)
                zest.block(i*9+j*3, 0,3,1) = rot.block(0, j, 3, 1);
        }
        else
            occlusionIdx.push_back(i);
    }

    // Faire le filtre
    iteration(IMUobs, zest, H, occlusionIdx);

    getState(Q, Qdot, Qddot);
}
