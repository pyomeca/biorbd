#ifndef S2MNODEMUSCLE_H
#define S2MNODEMUSCLE_H
#include <Eigen/Dense>
#include "s2mNode.h"
#include "biorbdConfig.h"


class s2mNode;
class BIORBD_API s2mNodeMuscle : public s2mNode
{
    public:
        s2mNodeMuscle(const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
                      const s2mString& = "", // Nom du noeud
                      const s2mString& = ""); //  Nom du parent

        virtual ~s2mNodeMuscle();
        // Get and Set

    protected:
    private:

};

#endif // S2MNODEMUSCLE_H
