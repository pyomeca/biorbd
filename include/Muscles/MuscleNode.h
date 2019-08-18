#ifndef BIORBD_MUSCLES_MUSCLE_NODE_H
#define BIORBD_MUSCLES_MUSCLE_NODE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd {
namespace muscles {

class BIORBD_API MuscleNode : public biorbd::utils::Node
{
public:
    MuscleNode(
            const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
            const biorbd::utils::String& = "", // Nom du noeud
            const biorbd::utils::String& = ""); //  Nom du parent

    virtual ~MuscleNode();

};

}}

#endif // BIORBD_MUSCLES_MUSCLE_NODE_H
