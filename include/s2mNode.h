#ifndef S2M_NODE_H
#define S2M_NODE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/String.h"

class s2mAttitude;
class BIORBD_API s2mNode : public Eigen::Vector3d
{
    public:
    s2mNode(double x, double y, double z, // Position du noeud
            const s2mString &name = "",  // Nom du noeud
            const s2mString &parentName = "");
    s2mNode(const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const s2mString &name = "",  // Nom du noeud
            const s2mString &parentName = "");

    const s2mString &parent() const;
    void setParent(const s2mString &parentName);

    // Get and Set
    void setPosition(const s2mNode& n);
    void setPosition(Eigen::Vector3d& n);
    void setPosition(Eigen::Vector4d& n);
    const s2mNode& position() const;
    Eigen::Vector3d vector() const;
    void setName(const s2mString &name);
    const s2mString& name() const;
    void applyRT(const s2mAttitude&);
    const s2mNode operator-(const s2mNode &) const; // overload d'opérateurs
    const s2mNode operator*(double) const; // overload d'opérateurs
    const s2mNode operator/(double) const; // overload d'opérateurs

    protected:
        s2mString m_parentName;
        s2mString m_markName;

};

#endif // S2M_NODE_H
