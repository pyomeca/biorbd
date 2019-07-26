#ifndef S2M_MATRIX_H
#define S2M_MATRIX_H
    #include "biorbdConfig.h"
    #include "s2mString.h"
    #include <Eigen/Dense>

class BIORBD_API s2mMatrix : public Eigen::MatrixXd
{
public:
    s2mMatrix();
    s2mMatrix(const Eigen::MatrixXd& m);
    s2mMatrix(unsigned int i, unsigned int j);

    s2mMatrix & operator= (Eigen::MatrixXd other);

    virtual Eigen::MatrixXd matrix() const;


protected:
private:

};

#endif // S2M_MATRIX_H
