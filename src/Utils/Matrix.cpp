#define BIORBD_API_EXPORTS
#include "Utils/Matrix.h"

#include "Utils/Vector.h"

biorbd::utils::Matrix::Matrix() :
    Eigen::MatrixXd()
{

}

biorbd::utils::Matrix::Matrix(unsigned int i, unsigned int j) :
    Eigen::MatrixXd(i, j)
{

}

biorbd::utils::Matrix biorbd::utils::Matrix::DeepCopy() const
{
    return  *this;
}

void biorbd::utils::Matrix::DeepCopy(const biorbd::utils::Matrix &other)
{
    *this = other;
}
