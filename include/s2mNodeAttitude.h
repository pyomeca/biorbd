#ifndef S2MNODEATTITUDE_H
#define S2MNODEATTITUDE_H
	#include "biorbdConfig.h"
    #include <Eigen/Dense>
    #include "s2mAttitude.h"
    #include "s2mVector.h"
    #include "s2mString.h"

class BIORBD_API s2mNodeAttitude : public s2mAttitude
{
    public:
    s2mNodeAttitude(const s2mAttitude& = s2mAttitude(), // Position du noeud
            const s2mString &name = "",  // Nom du noeud
            const s2mString &parentName = "");
    virtual ~s2mNodeAttitude();

    const s2mString& parent() const;
    void setParent(const s2mString &parentName);
    void setName(const s2mString &name);
    const s2mString& name() const;

    // Get and Set
    void setAttitude(const s2mAttitude&);
    const s2mAttitude& attitude() const;

    protected:
        s2mString m_parentName;
        s2mString m_RTName;
    private:

};

#endif // S2MNODEATTITUDE_H
