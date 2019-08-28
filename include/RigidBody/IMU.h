#ifndef BIORBD_RIGIDBODY_IMU_H
#define BIORBD_RIGIDBODY_IMU_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/RotoTransNode.h"

namespace biorbd {
namespace utils {
class String;
}

namespace rigidbody {

class BIORBD_API IMU : public biorbd::utils::RotoTransNode
{ 
public:
    IMU(
            bool isTechnical = true, // Si le marker est un marker technique
            bool isAnatomical = true);
    IMU(
            const biorbd::utils::RotoTransNode& RotoTrans, // Position
            bool isTechnical = true, // Si le marker est un marker technique
            bool isAnatomical = true); // Si le marker est un marker anatomique
    template<typename OtherDerived> IMU(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::RotoTransNode(other){}
    biorbd::rigidbody::IMU DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::IMU& other);

    // Get and Set
    // TODO Inherit isTechnical
    bool isTechnical() const;
    bool isAnatomical() const;
    template<typename OtherDerived>
        biorbd::rigidbody::IMU& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::RotoTransNode::operator=(other);
            return *this;
        }

protected:
    std::shared_ptr<bool> m_technical; // If a marker is a technical marker
    std::shared_ptr<bool> m_anatomical; // It marker is a anatomical marker

};

}}

#endif // BIORBD_RIGIDBODY_IMU_H
