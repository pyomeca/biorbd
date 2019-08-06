#ifndef S2MMUSCLEFORCE_H
#define S2MMUSCLEFORCE_H
    #include <Eigen/Dense>
    #include "biorbdConfig.h"
    #include "s2mMuscleGeometry.h"


class BIORBD_API s2mMuscleForce : public Eigen::Vector3d
{
    public:
        s2mMuscleForce();
        s2mMuscleForce(double x, double y, double z);
        s2mMuscleForce(const Eigen::Vector3d& force);
        s2mMuscleForce(const s2mMuscleGeometry& geo, double force);
        virtual ~s2mMuscleForce();

        // Get et set
        double norme() const;
        const Eigen::Vector3d& directionVector() const;
        virtual void setForce(const Eigen::Vector3d& force);
        virtual void setForce(double x, double y, double z);
        virtual void setForce(const s2mMuscleGeometry& geo, double force);
    protected:
    private:
        void computeNorm();
        double m_force;
//        Eigen::Vector3d m_direction;
};

#endif // S2MMUSCLEFORCE_H
