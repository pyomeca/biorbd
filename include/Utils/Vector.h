#ifndef BIORBD_UTILS_VECTOR_H
#define BIORBD_UTILS_VECTOR_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {

///
/// \brief Class Vector
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
    /// \brief Construct vector
    /// \param i TODO:
    ///
    Vector(unsigned int i);

    ///
    /// \brief Return the norm of the vector
    /// \param p TODO (p must be superior or equal to 2, default: 2)
    /// \param skipRoot To skip root (default: false)
    /// \return The norm of the vector
    ///
    double norm(
            unsigned int p = 2,
            bool skipRoot = false) const;
    ///
    /// \brief Return the norm gradient
    /// \param p TODO 
    /// \param skipRoot To skip root (default: false)
    /// \return The norm gradient
    ///
    biorbd::utils::Vector normGradient(
            unsigned int p = 2,
            bool skipRoot = false);

    /// 
    /// \brief To use operator "=" on vector
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
