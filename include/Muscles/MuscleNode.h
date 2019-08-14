#ifndef S2M_NODE_MUSCLE_H
#define S2M_NODE_MUSCLE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"

class BIORBD_API s2mNodeMuscle : public biorbd::utils::Node
{
public:
    s2mNodeMuscle(
            const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
            const biorbd::utils::String& = "", // Nom du noeud
            const biorbd::utils::String& = ""); //  Nom du parent

    virtual ~s2mNodeMuscle();

};

#endif // S2M_NODE_MUSCLE_H
