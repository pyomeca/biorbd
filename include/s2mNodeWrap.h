#ifndef S2M_NODE_WRAP_H
#define S2M_NODE_WRAP_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "s2mNode.h"

class BIORBD_API s2mNodeWrap : public s2mNode
{
    public:
        s2mNodeWrap(const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
                      const s2mString& = "", // Nom du noeud
                      const s2mString& = ""); //  Nom du parent

        virtual ~s2mNodeWrap();

};

#endif // S2M_NODE_WRAP_H
