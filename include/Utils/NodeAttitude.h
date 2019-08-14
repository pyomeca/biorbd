#ifndef S2M_NODE_ATTITUDE_H
#define S2M_NODE_ATTITUDE_H

#include "biorbdConfig.h"
#include "Utils/Attitude.h"
#include "Utils/String.h"

namespace biorbd { namespace utils {

class BIORBD_API NodeAttitude : public biorbd::utils::Attitude
{
public:
    NodeAttitude(
            const Attitude& = Attitude(), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~NodeAttitude();

    const biorbd::utils::String& parent() const;
    void setParent(const biorbd::utils::String &parentName);
    void setName(const biorbd::utils::String &name);
    const biorbd::utils::String& name() const;

    // Get and Set
    void setAttitude(const Attitude&);
    const Attitude& attitude() const;

protected:
    biorbd::utils::String m_parentName;
    biorbd::utils::String m_RTName;

};

}}

#endif // S2M_NODE_ATTITUDE_H
