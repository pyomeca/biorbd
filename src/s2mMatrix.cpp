#define BIORBD_API_EXPORTS
#include "s2mMatrix.h"

s2mMatrix::s2mMatrix() {}

s2mMatrix::s2mMatrix(const Eigen::MatrixXd &m) :
    Eigen::MatrixXd(m)
{

}

s2mMatrix::s2mMatrix(unsigned int i, unsigned int j) :
    Eigen::MatrixXd(i, j)
{

}

s2mMatrix::~s2mMatrix()
{

}

s2mMatrix &s2mMatrix::operator=(Eigen::MatrixXd other)
{
    swap(other);
    return *this;
}

Eigen::MatrixXd s2mMatrix::matrix() const{
    return  this->block(0,0,this->rows(),this->cols());
}
