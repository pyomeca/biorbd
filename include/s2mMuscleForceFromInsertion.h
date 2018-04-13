#ifndef S2MMUSCLEFORCEFROMINSERTION_H
#define S2MMUSCLEFORCEFROMINSERTION_H
    #include "s2mMuscleForce.h"


class s2mMuscleForceFromInsertion : public s2mMuscleForce
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

#endif // S2MMUSCLEFORCEFROMINSERTION_H
