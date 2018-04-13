#ifndef S2MMUSCLEFORCEFROMORIGIN_H
#define S2MMUSCLEFORCEFROMORIGIN_H
    #include "s2mMuscleForce.h"


class s2mMuscleForceFromOrigin : public s2mMuscleForce
{
    public:
        s2mMuscleForceFromOrigin();
        s2mMuscleForceFromOrigin(const double&, const double&, const double&);
        s2mMuscleForceFromOrigin(const Eigen::Vector3d&);
        s2mMuscleForceFromOrigin(const s2mMuscleGeometry&, const double&);
        ~s2mMuscleForceFromOrigin();

        // Get et set
        void setForce(const s2mMuscleGeometry&, const double&);
    protected:
    private:
};

#endif // S2MMUSCLEFORCEFROMORIGIN_H
