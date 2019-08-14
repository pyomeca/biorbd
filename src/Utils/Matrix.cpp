#define BIORBD_API_EXPORTS
#include "Utils/Matrix.h"

biorbd::utils::Matrix::Matrix() {}

biorbd::utils::Matrix::Matrix(const Eigen::MatrixXd &m) :
    Eigen::MatrixXd(m)
{

}

biorbd::utils::Matrix::Matrix(unsigned int i, unsigned int j) :
    Eigen::MatrixXd(i, j)
{

}

biorbd::utils::Matrix::~Matrix()
{

}

biorbd::utils::Matrix &biorbd::utils::Matrix::operator=(Eigen::MatrixXd other)
{
    swap(other);
    return *this;
}

Eigen::MatrixXd biorbd::utils::Matrix::matrix() const{
    return  this->block(0,0,this->rows(),this->cols());
}
