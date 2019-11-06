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
class Characteristics;
class State;

class BIORBD_API Muscle : public biorbd::muscles::Compound
{
public:
    Muscle();
    Muscle(
            const biorbd::utils::String& name, // Nom du muscle
            const biorbd::muscles::Geometry& position, // Position origine/insertion
            const biorbd::muscles::Characteristics& characteristics); // Set d'un état actuel au départ
    Muscle(
            const biorbd::utils::String& name, // Nom du muscle
            const biorbd::muscles::Geometry& position, // Position origine/insertion
            const biorbd::muscles::Characteristics& characteristics, // Set d'un état actuel au départ
            const biorbd::muscles::StateDynamics& dynamicState);
    Muscle(
            const biorbd::utils::String& name, // Nom du muscle
            const biorbd::muscles::Geometry& position, // Position origine/insertion
            const biorbd::muscles::Characteristics& characteristics, // Muscle characteristics
            const biorbd::muscles::PathChangers& wrap);
    Muscle(
            const biorbd::utils::String&, // Nom du muscle
            const biorbd::muscles::Geometry&, // Position origine/insertion
            const biorbd::muscles::Characteristics&, // Muscle characteristics
            const biorbd::muscles::PathChangers&, // Set de wrapping objects
            const biorbd::muscles::StateDynamics&); // Set d'un état actuel au départ
    Muscle(
            const biorbd::muscles::Muscle& muscle);
    Muscle(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);
    virtual ~Muscle();
    void DeepCopy(const biorbd::muscles::Muscle& other);

    // Get and set
    double length(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            int updateKin = 2);
    double musculoTendonLength(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            int updateKin = 2);
    double velocity(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            bool updateKin = true);
    void updateOrientations(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            int updateKin = 2); // Update de la position de ce muscle
    void updateOrientations(
            biorbd::rigidbody::Joints &model,
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
    const biorbd::muscles::Characteristics& characteristics() const;
    void setPosition(const biorbd::muscles::Geometry &val);
    void setCharacteristics(const biorbd::muscles::Characteristics &val);
    const std::vector<biorbd::utils::Node3d>& musclesPointsInGlobal(
            biorbd::rigidbody::Joints &j,
            const biorbd::rigidbody::GeneralizedCoordinates &Q);
    const std::vector<biorbd::utils::Node3d>& musclesPointsInGlobal() const;
    void forceIsoMax(double);

    // Get and set
    void setState(const biorbd::muscles::StateDynamics &s);
    const biorbd::muscles::StateDynamics& state() const;
    biorbd::muscles::StateDynamics& state();
    double activationDot(const biorbd::muscles::StateDynamics &s, bool =false);
protected:
    virtual void computeForce(const biorbd::muscles::State &emg); // Calcul des forces
    virtual double getForceFromActivation(const biorbd::muscles::State &emg) = 0;
    std::shared_ptr<biorbd::muscles::Geometry> m_position;
    std::shared_ptr<biorbd::muscles::Characteristics> m_characteristics;
    std::shared_ptr<biorbd::muscles::StateDynamics> m_state;

};

}}

#endif // BIORBD_MUSCLES_H
