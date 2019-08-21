#ifndef BIORBD_RIGIDBODY_IMU_H
#define BIORBD_RIGIDBODY_IMU_H

#include "biorbdConfig.h"
#include "Utils/NodeAttitude.h"

namespace biorbd {
namespace utils {
class String;
}

namespace rigidbody {

class BIORBD_API IMU : public biorbd::utils::NodeAttitude
{ 
public:
    IMU();
    IMU(
            const biorbd::utils::NodeAttitude& attitude, // Position
            bool isTechnical = true, // Si le marker est un marker technique
            bool isAnatomical = true); // Si le marker est un marker anatomique
    biorbd::rigidbody::IMU DeepCopy();

    // Get and Set
    bool isTechnical() const;
    bool isAnatomical() const;

protected:
    bool m_technical; // If a marker is a technical marker
    bool m_anatomical; // It marker is a anatomical marker

};

}}

#endif // BIORBD_RIGIDBODY_IMU_H
