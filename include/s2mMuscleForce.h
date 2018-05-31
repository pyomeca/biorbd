#ifndef S2MMUSCLEFORCE_H
#define S2MMUSCLEFORCE_H
    #include <Eigen/Dense>
    #include "biorbdConfig.h"
    #include "s2mMuscleGeometry.h"


class BIORBD_API s2mMuscleForce : public Eigen::Vector3d
{
    public:
        s2mMuscleForce();
        s2mMuscleForce(const double&, const double&, const double&);
        s2mMuscleForce(const Eigen::Vector3d&);
        s2mMuscleForce(const s2mMuscleGeometry&, const double&);
        ~s2mMuscleForce();

        // Get et set
        double norme() const { return m_force; }
        Eigen::Vector3d directionVector() const { return *this; }
        virtual void setForce(const Eigen::Vector3d&);
        virtual void setForce(const double&, const double&, const double&);
        virtual void setForce(const s2mMuscleGeometry&, const double&);
    protected:
    private:
        void computeNorm();
        double m_force;
//        Eigen::Vector3d m_direction;
};

#endif // S2MMUSCLEFORCE_H
