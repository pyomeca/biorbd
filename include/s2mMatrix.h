#ifndef S2MMATRIX_H
#define S2MMATRIX_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

class BIORBD_API s2mMatrix : public Eigen::MatrixXd
{
public:
    s2mMatrix();
    s2mMatrix(const Eigen::MatrixXd& m);
    s2mMatrix(unsigned int i, unsigned int j);
    virtual ~s2mMatrix();

    s2mMatrix & operator= (Eigen::MatrixXd other);

    virtual Eigen::MatrixXd matrix() const;

};

#endif // S2MMATRIX_H
