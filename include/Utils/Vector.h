#ifndef BIORBD_UTILS_VECTOR_H
#define BIORBD_UTILS_VECTOR_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {

///
/// \brief Wrapper of the Eigen VectorXd
///
class BIORBD_API Vector : public Eigen::VectorXd
{
public:
    ///
    /// \brief Construct vector
    ///
    Vector();

    ///
    /// \brief Construct vector from Eigen matrix
    /// \param other Eigen matrix
    ///
    template<typename OtherDerived> Vector(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::VectorXd(other){}

    ///
    /// \brief Construct vector of dimension size
    /// \param size The length of the vector
    ///
    Vector(
            unsigned int size);

    ///
    /// \brief Return the Euclidian p-norm of the vector
    /// \param p the factor of the p-norm
    /// \param skipRoot To perform or not the sqrt_p()
    /// \return The norm of the vector
    ///
    double norm(
            unsigned int p = 2,
            bool skipRoot = false) const;

    ///
    /// \brief Return the gradient of the p-norm
    /// \param p the factor of the p-norm
    /// \param skipRoot To perform or not the sqrt_p()
    /// \return The gradient of the norm
    ///
    biorbd::utils::Vector normGradient(
            unsigned int p = 2,
            bool skipRoot = false);

    /// 
    /// \brief Allow the use operator= on vector
    /// \param other The other matrix
    /// 
    template<typename OtherDerived>
        biorbd::utils::Vector& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::VectorXd::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_UTILS_VECTOR_H
