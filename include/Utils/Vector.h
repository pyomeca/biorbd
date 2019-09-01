#ifndef BIORBD_UTILS_VECTOR_H
#define BIORBD_UTILS_VECTOR_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {

class BIORBD_API Vector : public Eigen::VectorXd
{
public:
    Vector();
    template<typename OtherDerived> Vector(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::VectorXd(other){}
    Vector(unsigned int i);
    biorbd::utils::Vector DeepCopy() const;
    void DeepCopy(const biorbd::utils::Vector& other);

    double norm(
            unsigned int p = 2,
            bool skipRoot = false);
    biorbd::utils::Vector normGradient(
            unsigned int p = 2,
            bool skipRoot = false);

    template<typename OtherDerived>
        biorbd::utils::Vector& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::VectorXd::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_UTILS_VECTOR_H
