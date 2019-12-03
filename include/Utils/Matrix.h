#ifndef BIORBD_UTILS_MATRIX_H
#define BIORBD_UTILS_MATRIX_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
}

namespace utils {
///
/// \brief A wrapper for the Eigen::MatrixXd
///
class BIORBD_API Matrix : public Eigen::MatrixXd
{
public:
    ///
    /// \brief Construct matrix
    ///
    Matrix();

    ///
    /// \brief Construct matrix from another Eigen matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived> Matrix(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::MatrixXd(other){}

    ///
    /// \brief Construct matrix of size nbRows,nbCols
    /// \param nbRows Number of rows
    /// \param nbCols Number of columns
    ///
    Matrix(
            unsigned int nbRows,
            unsigned int nbCols);

    ///
    /// \brief To use operator= with matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::Matrix& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::MatrixXd::operator=(other);
            return *this;
        }

};

}}

#endif // BIORBD_UTILS_MATRIX_H
