#ifndef S2M_MUSCLE_FORCE_FROM_INSERTION_H
#define S2M_MUSCLE_FORCE_FROM_INSERTION_H
    #include "biorbdConfig.h"
    #include "s2mMuscleForce.h"


class BIORBD_API s2mMuscleForceFromInsertion : public s2mMuscleForce
{
public:
    s2mMuscleForceFromInsertion();
    s2mMuscleForceFromInsertion(double x, double y, double z);
    s2mMuscleForceFromInsertion(const Eigen::Vector3d& force);
    s2mMuscleForceFromInsertion(
            const s2mMuscleGeometry& geo,
            double force);
    virtual ~s2mMuscleForceFromInsertion();

    // Get et set
    virtual void setForce(const s2mMuscleGeometry& geo, double force);

};

#endif // S2M_MUSCLE_FORCE_FROM_INSERTION_H
