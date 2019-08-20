#ifndef BIORBD_UTILS_NODE_H
#define BIORBD_UTILS_NODE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace utils {
class Attitude;

class BIORBD_API Node : public Eigen::Vector3d
{
    public:
    Node(
            double x = 0,
            double y = 0,
            double z = 0, // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    Node(
            const Eigen::Vector3d &v, // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");

    const biorbd::utils::String &parent() const;
    void setParent(const biorbd::utils::String &parentName);

    // Get and Set
    void setPosition(const biorbd::utils::Node& n);
    void setPosition(Eigen::Vector3d& n);
    void setPosition(Eigen::Vector4d& n);
    const biorbd::utils::Node& position() const;
    Eigen::Vector3d vector() const;
    void setName(const biorbd::utils::String &name);
    const biorbd::utils::String& name() const;
    void applyRT(const Attitude&);
    const biorbd::utils::Node operator-(const biorbd::utils::Node &) const; // overload d'opérateurs
    const biorbd::utils::Node operator*(double) const; // overload d'opérateurs
    const biorbd::utils::Node operator/(double) const; // overload d'opérateurs

    protected:
        biorbd::utils::String m_parentName;
        biorbd::utils::String m_markName;

};

}}

#endif // BIORBD_UTILS_NODE_H
