#ifndef BIORBD_MUSCLES_MUSCLE_NODE_H
#define BIORBD_MUSCLES_MUSCLE_NODE_H

#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd {
namespace muscles {

class BIORBD_API MuscleNode : public biorbd::utils::Node
{
public:
    MuscleNode(
            double x = 0,
            double y = 0,
            double z = 0,
            const biorbd::utils::String& nodeName = "", // Nom du noeud
            const biorbd::utils::String& parentName= "");
    MuscleNode(
            const biorbd::utils::Node&, // Position
            const biorbd::utils::String& nodeName = "", // Nom du noeud
            const biorbd::utils::String& parentName= ""); //  Nom du parent

    virtual ~MuscleNode();

};

}}

#endif // BIORBD_MUSCLES_MUSCLE_NODE_H
