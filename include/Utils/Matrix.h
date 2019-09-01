#ifndef BIORBD_UTILS_MATRIX_H
#define BIORBD_UTILS_MATRIX_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
}

namespace utils {

class BIORBD_API Matrix : public Eigen::MatrixXd
{
public:
    Matrix();
    template<typename OtherDerived> Matrix(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::MatrixXd(other){}
    Matrix(unsigned int i, unsigned int j);
    biorbd::utils::Matrix DeepCopy() const;
    void DeepCopy(const biorbd::utils::Matrix& other);


    template<typename OtherDerived>
        biorbd::utils::Matrix& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::MatrixXd::operator=(other);
            return *this;
        }

};

}}

#endif // BIORBD_UTILS_MATRIX_H
