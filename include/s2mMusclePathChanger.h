#ifndef S2MMUSCLEPATHCHANGER_H
#define S2MMUSCLEPATHCHANGER_H
#include "s2mNodeMuscle.h"
#include "s2mString.h"

class s2mMusclePathChanger : public s2mNodeMuscle
{
    public:
        s2mMusclePathChanger(const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
                             const s2mString &name = "",  // Nom du noeud
                             const s2mString &parentName = "");
        ~s2mMusclePathChanger() ;

        // Set and get
        virtual s2mString type() = 0;

    protected:
    private:
};

#endif // S2MMUSCLEPATHCHANGER_H
