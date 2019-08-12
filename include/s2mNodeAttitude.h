#ifndef S2M_NODE_ATTITUDE_H
#define S2M_NODE_ATTITUDE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Attitude.h"
#include "Utils/String.h"

class BIORBD_API s2mNodeAttitude : public biorbd::utils::Attitude
{
public:
    s2mNodeAttitude(
            const biorbd::utils::Attitude& = biorbd::utils::Attitude(), // Position du noeud
            const s2mString &name = "",  // Nom du noeud
            const s2mString &parentName = "");
    virtual ~s2mNodeAttitude();

    const s2mString& parent() const;
    void setParent(const s2mString &parentName);
    void setName(const s2mString &name);
    const s2mString& name() const;

    // Get and Set
    void setAttitude(const Attitude&);
    const biorbd::utils::Attitude& attitude() const;

protected:
    s2mString m_parentName;
    s2mString m_RTName;

};

#endif // S2M_NODE_ATTITUDE_H
