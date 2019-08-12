#ifndef S2M_MUSCLE_FORCE_FROM_ORIGIN_H
#define S2M_MUSCLE_FORCE_FROM_ORIGIN_H
    #include "biorbdConfig.h"
    #include "s2mMuscleForce.h"


class BIORBD_API s2mMuscleForceFromOrigin : public s2mMuscleForce
{
    public:
        s2mMuscleForceFromOrigin();
        s2mMuscleForceFromOrigin(double x, double y, double z);
        s2mMuscleForceFromOrigin(const Eigen::Vector3d& force);
        s2mMuscleForceFromOrigin(
                const s2mMuscleGeometry& geo,
                double force);
        virtual ~s2mMuscleForceFromOrigin();

        // Get et set
        void setForce(const s2mMuscleGeometry& geo, double force);

};

#endif // S2M_MUSCLE_FORCE_FROM_ORIGIN_H
