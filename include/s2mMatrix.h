#ifndef S2MMATRIX_H
#define S2MMATRIX_H
    #include "s2mString.h"
    #include <Eigen/Dense>

class MatrixRT : public  Eigen::Matrix4d
{
public:
    MatrixRT(void):Eigen::Matrix4d() { setRT(*this); }
    typedef Eigen::Matrix4d Base;
    // This constructor allows you to construct MatrixRT from Eigen expressions
    template<typename OtherDerived>
    MatrixRT(const Eigen::MatrixBase<OtherDerived>& other)
        : Eigen::Matrix4d(other)
    { setRT(*this); }
    // This constructor allows you to construct MatrixRT from Eigen expressions
    MatrixRT(const Eigen::Matrix3d& R, const Eigen::Vector3d& T)
        { setRot(R); setTrans(T);}
//     This method allows you to assign Eigen expressions to MatrixRT
    template<typename OtherDerived>
    MatrixRT & operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }
    ~MatrixRT();

    // Réimplémentation de fonction
    // La transposée (et l'inverse) d'une rototrans est [ R^T, -R^T*T; 0 0 0 1]
    virtual MatrixRT transpose();
    virtual MatrixRT inverse();

    // Set and get
    virtual void setRT(const Eigen::Matrix4d &RT);
    virtual void setRT(const MatrixRT &RT);
    virtual void setTrans(const Eigen::Vector3d &T);
    virtual void setTrans(const Eigen::Matrix4d &RT);
    virtual void setRot(const Eigen::Matrix3d &R);
    virtual void setRot(const Eigen::Matrix4d &RT);
    virtual MatrixRT& getRT();
    virtual Eigen::Vector3d& getTrans();
    virtual Eigen::Matrix3d& getRot();

protected:
    Eigen::Vector3d m_T;
    Eigen::Matrix3d m_R;
};


#endif // S2MMATRIX_H
