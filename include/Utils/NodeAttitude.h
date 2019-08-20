#ifndef BIORBD_UTILS_NODE_ATTITUDE_H
#define BIORBD_UTILS_NODE_ATTITUDE_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/Attitude.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class String;

class BIORBD_API NodeAttitude : public biorbd::utils::Attitude, public biorbd::utils::Node
{
public:
    NodeAttitude();
    NodeAttitude(const Attitude& attitude);
    NodeAttitude(
            const Attitude& attitude,
            const biorbd::utils::String &name,
            const biorbd::utils::String &parentName);
    biorbd::utils::NodeAttitude DeepCopy();

    // Get and Set
    void setAttitude(const Attitude&);
    const Attitude& attitude() const;

};

}}

#endif // BIORBD_UTILS_NODE_ATTITUDE_H
