#define BIORBD_API_EXPORTS
#include "Utils/Matrix.h"

#include "Utils/Vector.h"

using namespace BIORBD_NAMESPACE;

utils::Matrix::Matrix() :
    RigidBodyDynamics::Math::MatrixNd()
{

}


#ifdef BIORBD_USE_CASADI_MATH

utils::Matrix::Matrix(
    const utils::Matrix& v) :
    RigidBodyDynamics::Math::MatrixNd (v)
{

}

utils::Matrix::Matrix(
    const RigidBodyDynamics::Math::MatrixNd &v) :
    RigidBodyDynamics::Math::MatrixNd (v)
{

}

utils::Matrix::Matrix(
    const RBDLCasadiMath::MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::VectorNd (m)
{

}

void utils::Matrix::operator=(
    const utils::Matrix &other)
{
    this->MX_Xd_dynamic::operator=(other);
}

void utils::Matrix::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->MX_Xd_dynamic::operator=(other);
}

#endif


utils::Matrix::Matrix(
    unsigned int nbRows,
    unsigned int nbCols) :
    RigidBodyDynamics::Math::MatrixNd(nbRows, nbCols)
{

}
