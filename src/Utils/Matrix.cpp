#define BIORBD_API_EXPORTS
#include "Utils/Matrix.h"

#include "Utils/Vector.h"

biorbd::utils::Matrix::Matrix() :
    Eigen::MatrixXd()
{

}

biorbd::utils::Matrix::Matrix(
        unsigned int nbRows,
        unsigned int nbCols) :
    Eigen::MatrixXd(nbRows, nbCols)
{

}
