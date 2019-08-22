#define BIORBD_API_EXPORTS
#include "RigidBody/KalmanRecons.h"

#include "BiorbdModel.h"
#include "RigidBody/GeneralizedCoordinates.h"

biorbd::rigidbody::KalmanRecons::KalmanRecons(
        biorbd::Model &model,
        unsigned int nMeasure,
        KalmanParam params) :
    m_params(params),
    m_Te(1.0/(m_params.acquisitionFrequency())),
    m_nDof(model.dof_count),
    m_nMeasure(nMeasure)
{

}

biorbd::rigidbody::KalmanRecons::~KalmanRecons()
{

}


void biorbd::rigidbody::KalmanRecons::iteration(
        biorbd::utils::Vector measure,
        const biorbd::utils::Vector &projectedMeasure,
        const biorbd::utils::Matrix &Hessian,
        const std::vector<unsigned int> &occlusion){
    // Prédiction
    biorbd::utils::Vector xkm(m_A * m_xp);
    biorbd::utils::Matrix Pkm(m_A * m_Pp * m_A.transpose() + m_Q);

    // Correction
    biorbd::utils::Matrix InvTp((Hessian*Pkm*Hessian.transpose() + m_R).inverse());
    manageOcclusionDuringIteration(InvTp, measure, occlusion);
    biorbd::utils::Matrix K(Pkm*Hessian.transpose() * InvTp); // Gain

    m_xp = biorbd::utils::Vector(xkm+K*(measure-projectedMeasure)); // New estimated state
    m_Pp =  (biorbd::utils::Matrix::Identity(3*m_nDof, 3*m_nDof) - K*Hessian) *
             Pkm *
            (biorbd::utils::Matrix::Identity(3*m_nDof, 3*m_nDof) - K*Hessian).transpose() +
            K*m_R*K.transpose();

}

void biorbd::rigidbody::KalmanRecons::manageOcclusionDuringIteration(
        biorbd::utils::Matrix &InvTp,
        biorbd::utils::Vector &measure,
        const std::vector<unsigned int> &occlusion){
    for (unsigned int i = 0; i < occlusion.size(); ++i)
         for (unsigned int j=occlusion[i]; j< occlusion[i]+1; ++j){
             InvTp(j,j) = 0; // Artéfact du au fait que m_R a une valeur à (j:j+2,j:j+2)
             measure(j) = 0;
         }
}

void biorbd::rigidbody::KalmanRecons::getState(
        biorbd::rigidbody::GeneralizedCoordinates *Q,
        biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        biorbd::rigidbody::GeneralizedCoordinates *Qddot){
    if (Q != nullptr)
        *Q = m_xp.block(0, 0, m_nDof, 1);

    if (Qdot != nullptr)
        *Qdot = m_xp.block(m_nDof, 0, m_nDof, 1);

    if (Qddot != nullptr)
        *Qddot = m_xp.block(2*m_nDof, 0, m_nDof, 1);
}


biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::evolutionMatrix(
        const unsigned int nQ,
        unsigned int n,
        double Te){
    // m  : nombre de degré de liberté
    // n  : ordre du développent de Taylor
    // Te : 1 / (frequence d'acquisition)

    n += 1;
    biorbd::utils::Matrix A(biorbd::utils::Matrix::Identity(nQ*n,nQ*n));
    double c = 1;
    for (unsigned int i=2; i<n+1; ++i){

        unsigned int j=(i-1) * nQ;
        c = c/(i-1);


        for (unsigned int cmp=0; cmp<nQ*n-j; ++cmp)
            A(0+cmp,j+cmp) += c* static_cast<double>(std::pow(Te,(static_cast<double>(i)-1.0)));
    }

    return A;
}

biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::processNoiseMatrix(
        const unsigned int nQ,
        double Te){

    // Trouver la valeur des coefficients
    double c1 = 1.0/20.0 * pow(Te,5);
    double c2 = 1.0/8.0  * pow(Te,4);
    double c3 = 1.0/6.0  * pow(Te,3);
    double c4 = 1.0/3.0  * pow(Te,3);
    double c5 = 1.0/2.0  * pow(Te,2);
    double c6 = Te;

    // Matrice de sortie
    biorbd::utils::Matrix Q(biorbd::utils::Matrix::Zero(3*nQ,3*nQ));
    for (unsigned int j=0; j<nQ; ++j){
        Q(     j,      j) = c1;
        Q(     j,   nQ+j) = c2;
        Q(     j, 2*nQ+j) = c3;
        Q(  nQ+j,      j) = c2;
        Q(  nQ+j,   nQ+j) = c4;
        Q(  nQ+j, 2*nQ+j) = c5;
        Q(2*nQ+j,      j) = c3;
        Q(2*nQ+j,   nQ+j) = c5;
        Q(2*nQ+j, 2*nQ+j) = c6;
    }

    return Q;
}

// Méthodes lors de l'initialisation
void biorbd::rigidbody::KalmanRecons::initialize(){

    // Déclaration de la matrice d'évolution
    m_A = evolutionMatrix(m_nDof, 2, m_Te);

    // Process Noise Matrix
    m_Q = processNoiseMatrix(m_nDof, m_Te);

    // Matrice de bruit sur la mesure
    m_R = measurementNoiseMatrix(m_nMeasure, m_params.noiseFactor());

    // Initialiser l'état
    m_xp = initState(m_nDof);

    // Matrice Pp
    m_Pp = initCovariance(m_nDof, m_params.errorFactor());
}


biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::measurementNoiseMatrix(
        const unsigned int nMeasure,
        double MN){
    biorbd::utils::Matrix R(biorbd::utils::Matrix::Zero(nMeasure, nMeasure));
    for (unsigned int i=0; i<nMeasure; ++i)
        R(i,i) = MN;
    return R;
}

biorbd::rigidbody::GeneralizedCoordinates biorbd::rigidbody::KalmanRecons::initState(const unsigned int nQ){
    return biorbd::rigidbody::GeneralizedCoordinates(biorbd::utils::Vector::Zero(3*nQ)); // Q, Qdot, Qddot
}

void biorbd::rigidbody::KalmanRecons::setInitState(
        const biorbd::rigidbody::GeneralizedCoordinates *Q,
        const biorbd::rigidbody::GeneralizedCoordinates *Qdot,
        const biorbd::rigidbody::GeneralizedCoordinates *Qddot){
    if (Q != nullptr)
        m_xp.block(0, 0, m_nDof, 1) = *Q;

    if (Qdot != nullptr)
        m_xp.block(m_nDof, 0, m_nDof, 1) = *Qdot;

    if (Qddot != nullptr)
        m_xp.block(2*m_nDof, 0, m_nDof, 1) = *Qddot;
}


biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::initCovariance(
        const unsigned int nQ,
        double csnt){
    biorbd::utils::Matrix Pp(biorbd::utils::Matrix::Zero(3*nQ, 3*nQ));
    for (unsigned int i=0; i<3*nQ; ++i)
        Pp(i,i) = csnt;
    return Pp;
}





biorbd::rigidbody::KalmanRecons::KalmanParam::KalmanParam(
        double frequency,
        double noiseFactor,
        double errorFactor):
    m_acquisitionFrequency(frequency),
    m_noiseFactor(noiseFactor),
    m_errorFactor(errorFactor){}

double biorbd::rigidbody::KalmanRecons::KalmanParam::acquisitionFrequency() const{
    return m_acquisitionFrequency;
}

double biorbd::rigidbody::KalmanRecons::KalmanParam::noiseFactor() const
{
    return m_noiseFactor;
}

double biorbd::rigidbody::KalmanRecons::KalmanParam::errorFactor() const
{
    return m_errorFactor;
}
