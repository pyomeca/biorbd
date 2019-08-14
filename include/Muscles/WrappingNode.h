#ifndef S2M_NODE_WRAP_H
#define S2M_NODE_WRAP_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"

class BIORBD_API s2mNodeWrap : public biorbd::utils::Node
{
public:
    s2mNodeWrap(
            const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
            const biorbd::utils::String& = "", // Nom du noeud
            const biorbd::utils::String& = ""); //  Nom du parent

    virtual ~s2mNodeWrap();

};

#endif // S2M_NODE_WRAP_H
