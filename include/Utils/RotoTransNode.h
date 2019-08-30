#ifndef BIORBD_UTILS_ROTO_TRANS_NODE_H
#define BIORBD_UTILS_ROTO_TRANS_NODE_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/RotoTrans.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class String;

class BIORBD_API RotoTransNode : public biorbd::utils::RotoTrans, public biorbd::utils::Node
{
public:
    RotoTransNode();
    template<typename OtherDerived> RotoTransNode(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::RotoTrans(other){}
    RotoTransNode(
            const RotoTrans& rt,
            const biorbd::utils::String &name,
            const biorbd::utils::String &parentName);
    biorbd::utils::RotoTransNode DeepCopy() const;
    void DeepCopy(const biorbd::utils::RotoTransNode& other);
    template<typename OtherDerived>
        biorbd::utils::RotoTransNode& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::RotoTrans::operator=(other);
            return *this;
        }

protected:
    void setType();

};

}}

#endif // BIORBD_UTILS_ROTO_TRANS_NODE_H
