#ifndef S2M_MUSCLE_FORCE_FROM_INSERTION_H
#define S2M_MUSCLE_FORCE_FROM_INSERTION_H
    #include "biorbdConfig.h"
    #include "s2mMuscleForce.h"


class BIORBD_API s2mMuscleForceFromInsertion : public s2mMuscleForce
{
    public:
        s2mMuscleForceFromInsertion();
        s2mMuscleForceFromInsertion(const double&, const double&, const double&);
        s2mMuscleForceFromInsertion(const Eigen::Vector3d&);
        s2mMuscleForceFromInsertion(const s2mMuscleGeometry&, const double&);
        ~s2mMuscleForceFromInsertion();

        // Get et set
        void setForce(const s2mMuscleGeometry&, const double&);
    protected:
    private:
};

#endif // S2M_MUSCLE_FORCE_FROM_INSERTION_H
