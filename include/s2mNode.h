#ifndef S2MNODE_H
#define S2MNODE_H
	#include "biorbdConfig.h"
    #include <Eigen/Dense>
    #include "s2mAttitude.h"
    #include "s2mString.h"

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
    ~s2mNode();

    virtual s2mString parent() const;
    virtual void setParent(const s2mString &parentName);

    // Get and Set
    virtual void setPosition(const s2mNode& n);
    virtual void setPosition(Eigen::Vector3d& n);
    virtual void setPosition(Eigen::Vector4d& n);
    virtual s2mNode position() const;
    virtual Eigen::Vector3d vector() const;
    virtual void setName(const s2mString &name);
    virtual s2mString name() const;
    virtual void applyRT(const s2mAttitude&);
    const s2mNode operator-(const s2mNode &) const; // overload d'opérateurs
    const s2mNode operator*(double) const; // overload d'opérateurs
    const s2mNode operator/(double) const; // overload d'opérateurs

    protected:
        s2mString m_parentName;
        s2mString m_markName;
    private:

};

#endif // S2MNODE_H
