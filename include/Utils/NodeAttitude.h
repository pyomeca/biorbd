#ifndef BIORBD_UTILS_NODE_ATTITUDE_H
#define BIORBD_UTILS_NODE_ATTITUDE_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/Attitude.h"

namespace biorbd {
namespace utils {
class String;

class BIORBD_API NodeAttitude : public biorbd::utils::Attitude
{
public:
    NodeAttitude(
            const Attitude&, // Position du noeud
            const biorbd::utils::String &name,  // Nom du noeud
            const biorbd::utils::String &parentName);
    virtual ~NodeAttitude();

    const biorbd::utils::String& parent() const;
    void setParent(const biorbd::utils::String &parentName);
    void setName(const biorbd::utils::String &name);
    const biorbd::utils::String& name() const;

    // Get and Set
    void setAttitude(const Attitude&);
    const Attitude& attitude() const;

protected:
    std::shared_ptr<biorbd::utils::String> m_parentName;
    std::shared_ptr<biorbd::utils::String> m_RTName;

};

}}

#endif // BIORBD_UTILS_NODE_ATTITUDE_H
