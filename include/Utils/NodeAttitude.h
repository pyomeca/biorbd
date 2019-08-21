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
    NodeAttitude(
            double e00, double e01, double e02, double e03,
            double e10, double e11, double e12, double e13,
            double e20, double e21, double e22, double e23,
            double e30, double e31, double e32, double e33);
    NodeAttitude(
            double e00, double e01, double e02, double e03,
            double e10, double e11, double e12, double e13,
            double e20, double e21, double e22, double e23,
            double e30, double e31, double e32, double e33,
            const biorbd::utils::String &name,
            const biorbd::utils::String &parentName);
    NodeAttitude(const Attitude& attitude);
    NodeAttitude(
            const Attitude& attitude,
            const biorbd::utils::String &name,
            const biorbd::utils::String &parentName);
    biorbd::utils::NodeAttitude DeepCopy() const;

};

}}

#endif // BIORBD_UTILS_NODE_ATTITUDE_H
