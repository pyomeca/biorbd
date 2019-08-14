#ifndef BIORBD_RIGIDBODY_IMU_H
#define BIORBD_RIGIDBODY_IMU_H

#include "biorbdConfig.h"
#include "Utils/NodeAttitude.h"

namespace biorbd {
namespace rigidbody {

class BIORBD_API IMU : public biorbd::utils::NodeAttitude
{ 
public:
    IMU(
            const biorbd::utils::Attitude& = biorbd::utils::Attitude(), // Position
            const biorbd::utils::String& = "", // Nom du noeud
            const biorbd::utils::String& = "", // Nom du parent
            const bool& = true, // Si le marker est un marker technique
            const bool& = true, // Si le marker est un marker anatomique
            const int& = -1); // Numéro ID du parent

    virtual ~IMU();
    // Get and Set
    virtual bool isTechnical() const;
    virtual bool isAnatomical() const;
    int parentId() const;
protected:
    bool m_technical; // If a marker is a technical marker
    bool m_anatomical; // It marker is a anatomical marker
    int m_id;

};

}}

#endif // BIORBD_RIGIDBODY_IMU_H
