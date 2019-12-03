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
/// \brief Class Matrix
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
    /// \brief Construct matrix of size i,j
    /// \param i Number of rows
    /// \param j Number of columns
    ///
    Matrix(unsigned int i, unsigned int j);

    ///
    /// \brief To use operator "=" with matrix
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
