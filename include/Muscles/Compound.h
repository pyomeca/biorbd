#ifndef BIORBD_MUSCLES_COMPOUND_H
#define BIORBD_MUSCLES_COMPOUND_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Node3d;
}

namespace rigidbody {
class Joints;
class GeneralizedCoordinates;
}

namespace muscles {
class Force;
class Caracteristics;
class PathChangers;
class StateDynamics;

class BIORBD_API Compound
{
public:
    Compound();
    Compound(
            const biorbd::utils::String &name);
    Compound(
            const biorbd::utils::String &name,
            const biorbd::muscles::PathChangers&);
    Compound(
            const biorbd::muscles::Compound& muscle);
    Compound(
            const std::shared_ptr<biorbd::muscles::Compound> muscle);
    virtual ~Compound();
    void DeepCopy(const biorbd::muscles::Compound& other);

    const biorbd::utils::String& name() const;
    void setName(const biorbd::utils::String& name);
    const biorbd::utils::String& type() const;

    // Wrapping object
    const biorbd::muscles::PathChangers& pathChanger();
    void addPathObject(biorbd::utils::Node3d &w); // Ajouter un wrapping object

    virtual const std::vector<biorbd::muscles::Force>& force(); // Return the last computed muscle force
    virtual const std::vector<biorbd::muscles::Force>& force(const biorbd::muscles::StateDynamics& emg) = 0;
    virtual const std::vector<biorbd::muscles::Force>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2) = 0;
    virtual const std::vector<biorbd::muscles::Force>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2) = 0;

protected:
    std::shared_ptr<biorbd::utils::String> m_name;
    std::shared_ptr<biorbd::utils::String> m_type;
    std::shared_ptr<biorbd::muscles::PathChangers> m_pathChanger;
    std::shared_ptr<std::vector<biorbd::muscles::Force>> m_force;
    void copyForce(const std::shared_ptr<std::vector<biorbd::muscles::Force>>& force);
    virtual void setType()=0;

};

}}

#endif // BIORBD_MUSCLES_COMPOUND_H
