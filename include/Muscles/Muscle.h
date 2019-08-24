#ifndef BIORBD_MUSCLES_H
#define BIORBD_MUSCLES_H

#include "biorbdConfig.h"
#include "Muscles/Compound.h"

namespace biorbd {
namespace utils {
class Matrix;
class Node3d;
}

namespace muscles {
class Geometry;
class Caracteristics;
class StateDynamics;

class BIORBD_API Muscle : public biorbd::muscles::Compound
{
public:
    Muscle();
    Muscle(
            const biorbd::utils::String& name, // Nom du muscle
            const biorbd::muscles::Geometry& position, // Position origine/insertion
            const biorbd::muscles::Caracteristics& caract); // Set d'un état actuel au départ

    Muscle(
            const biorbd::utils::String&, // Nom du muscle
            const biorbd::muscles::Geometry&, // Position origine/insertion
            const biorbd::muscles::Caracteristics&, // Caractéristiques du muscle
            const biorbd::muscles::PathChangers&, // Set de wrapping objects
            const biorbd::muscles::StateDynamics&); // Set d'un état actuel au départ
    Muscle(
            const biorbd::muscles::Muscle& muscle);
    Muscle(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);
    virtual ~Muscle();

    // Get and set
    double length(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&, int = 2);
    double musculoTendonLength(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&, int = 2);
    double velocity(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            bool = true);
    void updateOrientations(
            biorbd::rigidbody::Joints &m,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            int updateKin = 2); // Update de la position de ce muscle
    void updateOrientations(
            biorbd::rigidbody::Joints &m,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            int updateKin = 2); // Update de la position de ce muscle
    void updateOrientations(
            std::vector<biorbd::utils::Node3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal); // Update de la position de ce muscle
    void updateOrientations(
            std::vector<biorbd::utils::Node3d>& musclePointsInGlobal,
            biorbd::utils::Matrix& jacoPointsInGlobal,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot); // Update de la position de ce muscle

    const biorbd::muscles::Geometry& position() const;
    const biorbd::muscles::Caracteristics& caract() const;
    void setPosition(const biorbd::muscles::Geometry &val);
    void setCaract(const biorbd::muscles::Caracteristics &val);
    const std::vector<biorbd::utils::Node3d>& musclesPointsInGlobal(
            biorbd::rigidbody::Joints &j,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true);
    void forceIsoMax(double);

    // Get and set
    void setState(const biorbd::muscles::StateDynamics &s);
    const biorbd::muscles::StateDynamics& state() const;
    biorbd::muscles::StateDynamics& state_nonConst() const;
    double activationDot(const biorbd::muscles::StateDynamics &s, bool =false);
protected:
    std::shared_ptr<biorbd::muscles::Geometry> m_position;
    std::shared_ptr<biorbd::muscles::Caracteristics> m_caract;
    std::shared_ptr<biorbd::muscles::StateDynamics> m_state;

};

}}

#endif // BIORBD_MUSCLES_H
