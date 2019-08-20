#ifndef BIORBD_MUSCLES_WRAPPING_NODE_H
#define BIORBD_MUSCLES_WRAPPING_NODE_H

#include "biorbdConfig.h"
#include "Muscles/PathChanger.h"

namespace biorbd {
namespace muscles {

class BIORBD_API WrappingNode : public biorbd::muscles::PathChanger
{
public:
    WrappingNode(
            double x,
            double y,
            double z,
            const biorbd::utils::String& nodeName = "", // Nom du noeud
            const biorbd::utils::String& parentName = ""); //  Nom du parent
    WrappingNode(
            const biorbd::utils::Node3d& node,
            const biorbd::utils::String& nodeName = "", // Nom du noeud
            const biorbd::utils::String& parentName = ""); //  Nom du parent

    virtual ~WrappingNode();

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_NODE_H
