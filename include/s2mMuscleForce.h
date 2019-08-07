#ifndef S2M_MUSCLE_FORCE_H
#define S2M_MUSCLE_FORCE_H
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

};

#endif // S2M_MUSCLE_FORCE_H
