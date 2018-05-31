#define BIORBD_API_EXPORTS
#include "../include/s2mMatrix.h"


MatrixRT::~MatrixRT(){

}

// Get et set Trans
void MatrixRT::setTrans(const Eigen::Vector3d &T) {
    MatrixRT RT(*this);
    for (unsigned int i=0;i<3;i++){
        RT(i,3) = T(i);
        m_T(i) = T(i);
    }
    *this = RT;
}
void MatrixRT::setTrans(const Eigen::Matrix4d &T) {
    Eigen::Matrix4d RT(*this);
    for (unsigned int i=0;i<3;i++){
        RT(i,3) = T(i,3);
        m_T(i) = T(i,3);
    }
    *this = RT;
}
Eigen::Vector3d& MatrixRT::getTrans(){
    return m_T;
}

// Get et Set Rot
void MatrixRT::setRot(const Eigen::Matrix3d &R) {
    Eigen::Matrix4d RT(*this);
    for (unsigned int i=0;i<3;i++)
        for (unsigned int j=0;j<3;j++){
            RT(i,j) = R(i,j);
            m_R(i,j) = R(i,j);
        }
    *this = RT;
}
void MatrixRT::setRot(const Eigen::Matrix4d &R) {
    Eigen::Matrix4d RT(*this);
    for (unsigned int i=0;i<3;i++)
        for (unsigned int j=0;j<3;j++){
            RT(i,j) = R(i,j);
            m_R(i,j) = R(i,j);
        }
    *this = RT;
}
Eigen::Matrix3d& MatrixRT::getRot(){
    return m_R;
}

// Get et Set RT
void MatrixRT::setRT(const Eigen::Matrix4d &RT){
    *this = RT;
    setRot(RT);
    setTrans(RT);
}
void MatrixRT::setRT(const MatrixRT &RT){
    *this = RT;
    setRot(*this);
    setTrans(*this);
}
MatrixRT& MatrixRT::getRT(){
    return *this;
}


// Réimplémentation de fonction
MatrixRT MatrixRT::transpose(){
    // La transposée d'une rototrans est [ R^T, -R^T*T; 0 0 0 1]

    Eigen::Matrix4d RT(*this), RT2;
    setRot(*this);
    setTrans(*this);
    Eigen::Vector3d T = -m_R.transpose() *m_T;

    RT2=RT.transpose(); // Transposer simplement la partie 3*3
    for (unsigned int i=0; i<3; i++){
        RT2(3,i) = 0;
        RT2(i,3) = T(i);
    }
    RT2(3,3) = 1;

    return RT2;
}
MatrixRT MatrixRT::inverse(){
    return transpose();
}
