#ifndef BIORBD_UTILS_MATRIX_H
#define BIORBD_UTILS_MATRIX_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd { namespace utils {

class BIORBD_API Matrix : public Eigen::MatrixXd
{
public:
    Matrix();
    Matrix(const Eigen::MatrixXd& m);
    Matrix(unsigned int i, unsigned int j);
    virtual ~Matrix();

    biorbd::utils::Matrix & operator= (Eigen::MatrixXd other);

    virtual Eigen::MatrixXd matrix() const;

};

}}

#endif // BIORBD_UTILS_MATRIX_H
