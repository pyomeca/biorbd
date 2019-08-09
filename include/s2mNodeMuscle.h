#ifndef S2M_NODE_MUSCLE_H
#define S2M_NODE_MUSCLE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "s2mNode.h"

class BIORBD_API s2mNodeMuscle : public s2mNode
{
    public:
        s2mNodeMuscle(const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
                      const s2mString& = "", // Nom du noeud
                      const s2mString& = ""); //  Nom du parent

        virtual ~s2mNodeMuscle();

};

#endif // S2M_NODE_MUSCLE_H
