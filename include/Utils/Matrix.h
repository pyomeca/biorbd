#ifndef BIORBD_UTILS_MATRIX_H
#define BIORBD_UTILS_MATRIX_H

#include "biorbdConfig.h"
#include "rbdl_math.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
}

namespace utils {
///
/// \brief A wrapper for the Eigen::MatrixXd
///
class BIORBD_API Matrix : public RigidBodyDynamics::Math::MatrixNd
{
public:
    ///
    /// \brief Construct matrix
    ///
    Matrix();

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct matrix from another Eigen matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived> Matrix(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::MatrixXd(other){}
#endif

    ///
    /// \brief Construct matrix of size nbRows,nbCols
    /// \param nbRows Number of rows
    /// \param nbCols Number of columns
    ///
    Matrix(
            unsigned int nbRows,
            unsigned int nbCols);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief To use operator= with matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::Matrix& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::MatrixXd::operator=(other);
            return *this;
        }
#endif
};

}}

#endif // BIORBD_UTILS_MATRIX_H
