#ifndef S2MMUSCLEPATHCHANGER_H
#define S2MMUSCLEPATHCHANGER_H
#include "biorbdConfig.h"
#include "s2mNodeMuscle.h"
#include "s2mString.h"

class BIORBD_API s2mMusclePathChanger : public s2mNodeMuscle
{
    public:
        s2mMusclePathChanger(const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
                             const s2mString &name = "",  // Nom du noeud
                             const s2mString &parentName = "");
        virtual ~s2mMusclePathChanger() = 0;
        const s2mString& type() const;

    protected:
        s2mString m_type;
};

#endif // S2MMUSCLEPATHCHANGER_H
