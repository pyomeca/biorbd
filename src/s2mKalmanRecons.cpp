#define BIORBD_API_EXPORTS
#include "../include/s2mKalmanRecons.h"

s2mKalmanRecons::s2mKalmanRecons(s2mMusculoSkeletalModel &m, unsigned int nMeasure, s2mKalmanParam params) :
    m_params(params),
    m_Te(1.0/(m_params.acquisitionFrequency())),
    m_nDof(m.dof_count),
    m_nMeasure(nMeasure)
{

}

s2mKalmanRecons::~s2mKalmanRecons()
{

}


void s2mKalmanRecons::iteration(Eigen::VectorXd measure, const Eigen::VectorXd &projectedMeasure, const s2mMatrix &Hessian, const std::vector<unsigned int> &occlusion){
    // Prédiction
    Eigen::VectorXd xkm = m_A * m_xp;
    s2mMatrix Pkm(m_A * m_Pp * m_A.transpose() + m_Q);

    // Correction
    s2mMatrix InvTp((Hessian*Pkm*Hessian.transpose() + m_R).inverse());
    manageOcclusionDuringIteration(InvTp, measure, occlusion);
    s2mMatrix K(Pkm*Hessian.transpose() * InvTp); // Gain

    m_xp = xkm+K*(measure-projectedMeasure); // New estimated state
    m_Pp =  (s2mMatrix::Identity(3*m_nDof, 3*m_nDof) - K*Hessian) *
             Pkm *
            (s2mMatrix::Identity(3*m_nDof, 3*m_nDof) - K*Hessian).transpose() +
            K*m_R*K.transpose();

}

void s2mKalmanRecons::manageOcclusionDuringIteration(s2mMatrix &InvTp, Eigen::VectorXd &measure, const std::vector<unsigned int> &occlusion){
    for (unsigned int i = 0; i < occlusion.size(); ++i)
         for (unsigned int j=occlusion[i]; j< occlusion[i]+1; ++j){
             InvTp(j,j) = 0; // Artéfact du au fait que m_R a une valeur à (j:j+2,j:j+2)
             measure[j] = 0;
         }
}

void s2mKalmanRecons::getState(s2mGenCoord *Q, s2mGenCoord *Qdot, s2mGenCoord *Qddot){
    if (Q != nullptr)
        *Q = m_xp.block(0,0,m_nDof,1);

    if (Qdot != nullptr)
        *Qdot = m_xp.block(m_nDof,0,m_nDof,1);

    if (Qddot != nullptr)
        *Qddot = m_xp.block(2*m_nDof,0,m_nDof,1);
}


s2mMatrix s2mKalmanRecons::evolutionMatrix(const unsigned int nQ, unsigned int n, const double Te){
    // m  : nombre de degré de liberté
    // n  : ordre du développent de Taylor
    // Te : 1 / (frequence d'acquisition)

    n += 1;
    s2mMatrix A(s2mMatrix::Identity(nQ*n,nQ*n));
    double c = 1;
    for (unsigned int i=2; i<n+1; ++i){

        double j=(i-1) * nQ;
        c = c/(i-1);


        for (int cmp=0; cmp<nQ*n-j; ++cmp)
            A(int(0+cmp),int(j+cmp)) += c* static_cast<double>(std::pow(Te,(static_cast<double>(i)-1.0)));
    }

    return A;
}

s2mMatrix s2mKalmanRecons::processNoiseMatrix(const unsigned int nQ, const double Te){

    // Trouver la valeur des coefficients
    double c1 = 1.0/20.0 * pow(Te,5);
    double c2 = 1.0/8.0  * pow(Te,4);
    double c3 = 1.0/6.0  * pow(Te,3);
    double c4 = 1.0/3.0  * pow(Te,3);
    double c5 = 1.0/2.0  * pow(Te,2);
    double c6 = Te;

    // Matrice de sortie
    s2mMatrix Q(s2mMatrix::Zero(3*nQ,3*nQ));
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
void s2mKalmanRecons::initialize(){

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


s2mMatrix s2mKalmanRecons::measurementNoiseMatrix(const unsigned int nMeasure, const double MN){
    s2mMatrix R(s2mMatrix::Zero(nMeasure, nMeasure));
    for (unsigned int i=0; i<nMeasure; ++i)
        R(i,i) = MN;
    return R;
}

s2mGenCoord s2mKalmanRecons::initState(const unsigned int nQ){
    return s2mGenCoord(Eigen::VectorXd::Zero(3*nQ)); // Q, Qdot, Qddot
}

void s2mKalmanRecons::setInitState(const s2mGenCoord *Q, const s2mGenCoord *Qdot, const s2mGenCoord *Qddot){
    if (Q != nullptr)
        m_xp.block(0,0,m_nDof,1) = *Q;

    if (Qdot != nullptr)
        m_xp.block(m_nDof,0,m_nDof,1) = *Qdot;

    if (Qddot != nullptr)
        m_xp.block(2*m_nDof,0,m_nDof,1) = *Qddot;
}


s2mMatrix s2mKalmanRecons::initCovariance(const unsigned int nQ, const double csnt){
    s2mMatrix Pp(s2mMatrix::Zero(3*nQ, 3*nQ));
    for (unsigned int i=0; i<3*nQ; ++i)
        Pp(i,i) = csnt;
    return Pp;
}





s2mKalmanRecons::s2mKalmanParam::s2mKalmanParam(double frequency, double noiseFactor, double errorFactor):
    m_acquisitionFrequency(frequency),
    m_noiseFactor(noiseFactor),
    m_errorFactor(errorFactor){}

double s2mKalmanRecons::s2mKalmanParam::acquisitionFrequency() const{
    return m_acquisitionFrequency;
}

double s2mKalmanRecons::s2mKalmanParam::noiseFactor() const
{
    return m_noiseFactor;
}

double s2mKalmanRecons::s2mKalmanParam::errorFactor() const
{
    return m_errorFactor;
}
