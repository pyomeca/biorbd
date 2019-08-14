#ifndef BIORBD_MUSCLES_H
#define BIORBD_MUSCLES_H

#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Muscles/Compound.h"
#include "Muscles/Geometry.h"
#include "Muscles/Caracteristics.h"
#include "Muscles/StateDynamics.h"

namespace biorbd { namespace muscles {

class BIORBD_API Muscle : public biorbd::muscles::Compound
{
public:
    Muscle(
            const biorbd::utils::String& = "", // Nom du muscle
            const biorbd::muscles::Geometry& = biorbd::muscles::Geometry(), // Position origine/insertion
            const biorbd::muscles::Caracteristics& = biorbd::muscles::Caracteristics(), // Caractéristiques du muscle
            const biorbd::muscles::PathChangers& = biorbd::muscles::PathChangers(), // Set de wrapping objects
            const biorbd::muscles::StateDynamics& = biorbd::muscles::StateDynamics()); // Set d'un état actuel au départ
    Muscle(const Muscle& m);
    virtual ~Muscle();

    // Get and set
    double length(
            biorbd::rigidbody::Joints&,
            const biorbd::utils::GenCoord&, int = 2);
    double musculoTendonLength(
            biorbd::rigidbody::Joints&,
            const biorbd::utils::GenCoord&, int = 2);
    double velocity(
            biorbd::rigidbody::Joints&,
            const biorbd::utils::GenCoord&,
            const biorbd::utils::GenCoord&,
            const bool = true);
    void updateOrientations(
            biorbd::rigidbody::Joints &m,
            const biorbd::utils::GenCoord &Q,
            int updateKin = 2); // Update de la position de ce muscle
    void updateOrientations(
            biorbd::rigidbody::Joints &m,
            const biorbd::utils::GenCoord &Q,
            const biorbd::utils::GenCoord &Qdot,
            int updateKin = 2); // Update de la position de ce muscle
    void updateOrientations(
            std::vector<biorbd::muscles::MuscleNode>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal); // Update de la position de ce muscle
    void updateOrientations(
            std::vector<biorbd::muscles::MuscleNode>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal,
            const biorbd::utils::GenCoord &Qdot); // Update de la position de ce muscle

    const biorbd::muscles::Geometry& position() const;
    const biorbd::muscles::Caracteristics& caract() const;
    void setPosition(const biorbd::muscles::Geometry &val);
    void setCaract(const biorbd::muscles::Caracteristics &val);
    const std::vector<biorbd::muscles::MuscleNode>& musclesPointsInGlobal(
            biorbd::rigidbody::Joints &j,
            const biorbd::utils::GenCoord &Q,
            const bool updateKin = true);
    void forceIsoMax(double);

    // Get and set
    void setState(const biorbd::muscles::StateDynamics &s);
    const biorbd::muscles::StateDynamics& state() const;
    biorbd::muscles::StateDynamics& state_nonConst() const;
    double activationDot(const biorbd::muscles::StateDynamics &s, const bool =false);
protected:
    biorbd::muscles::Geometry m_position;
    biorbd::muscles::Caracteristics m_caract;
    biorbd::muscles::StateDynamics * m_state;

};

}}

#endif // BIORBD_MUSCLES_H
