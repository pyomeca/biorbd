#ifndef BIORBD_UTILS_VECTOR_H
#define BIORBD_UTILS_VECTOR_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd { namespace utils {

class BIORBD_API Vector : public Eigen::Vector3d
{
public:
    Vector();
    Vector(const Eigen::VectorXd& v);
    Vector(const biorbd::utils::Vector& v);
    Vector(unsigned int i);
    virtual ~Vector();
    Eigen::VectorXd vector() const;
    double norm(
            unsigned int p = 2,
            bool skipRoot = false);
    biorbd::utils::Vector normGradient(
            unsigned int p = 2,
            bool skipRoot = false);

};

}}

#endif // BIORBD_UTILS_VECTOR_H
