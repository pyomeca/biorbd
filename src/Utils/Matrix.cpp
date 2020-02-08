#define BIORBD_API_EXPORTS
#include "Utils/Matrix.h"

#include "Utils/Vector.h"

biorbd::utils::Matrix::Matrix() :
    RigidBodyDynamics::Math::MatrixNd()
{

}

biorbd::utils::Matrix::Matrix(
        unsigned int nbRows,
        unsigned int nbCols) :
    RigidBodyDynamics::Math::MatrixNd(nbRows, nbCols)
{

}
