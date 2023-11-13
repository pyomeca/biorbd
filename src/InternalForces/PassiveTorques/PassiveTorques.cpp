#define BIORBD_API_EXPORTS

#include <vector>
#include "Utils/Error.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/Joints.h"
#include "InternalForces/PassiveTorques/PassiveTorque.h"
#include "InternalForces/PassiveTorques/PassiveTorqueConstant.h"
#include "InternalForces/PassiveTorques/PassiveTorqueLinear.h"
#include "InternalForces/PassiveTorques/PassiveTorqueExponential.h"
#include "InternalForces/PassiveTorques/PassiveTorques.h"

using namespace BIORBD_NAMESPACE;

internal_forces::passive_torques::PassiveTorques::PassiveTorques() :
    m_pas(std::make_shared<std::vector<std::shared_ptr<internal_forces::passive_torques::PassiveTorque>>>()),
    m_isDofSet(std::make_shared<std::vector<bool>>(true))
{
    (*m_isDofSet)[0] = false;
}

internal_forces::passive_torques::PassiveTorques::PassiveTorques(
    const internal_forces::passive_torques::PassiveTorques& other) :
    m_pas(other.m_pas),
    m_isDofSet(other.m_isDofSet)
{

}

internal_forces::passive_torques::PassiveTorques::~PassiveTorques()
{

}

void internal_forces::passive_torques::PassiveTorques::DeepCopy(
        const internal_forces::passive_torques::PassiveTorques &other)
{
    m_pas->resize(other.m_pas->size());
    for (size_t i=0; i<other.m_pas->size(); ++i) {
        if ((*other.m_pas)[i]->type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_CONSTANT) {
            (*m_pas)[i] = std::make_shared<internal_forces::passive_torques::PassiveTorqueConstant>
                    (static_cast<const internal_forces::passive_torques::PassiveTorqueConstant&>(*(*other.m_pas)[i]));
            return;
        } else if ((*other.m_pas)[i]->type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_LINEAR) {
            (*m_pas)[i] = std::make_shared<internal_forces::passive_torques::PassiveTorqueLinear>
                    (static_cast<const internal_forces::passive_torques::PassiveTorqueLinear&>(*(*other.m_pas)[i]));
            return;
        } else if ((*other.m_pas)[i]->type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_EXPONENTIAL) {
            (*m_pas)[i] = std::make_shared<internal_forces::passive_torques::PassiveTorqueExponential>
                    (static_cast<const internal_forces::passive_torques::PassiveTorqueExponential&>(*(*other.m_pas)[i]));
            return;
        }
    }
    *m_isDofSet = *other.m_isDofSet;
}


void internal_forces::passive_torques::PassiveTorques::addPassiveTorque(
        const internal_forces::passive_torques::PassiveTorque &other)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Verify that the target dof is associated to a dof that already exists in the model
    utils::Error::check(
        other.index()<model.nbDof(), "Sent index is out of dof range");

    // For speed purposes and coherence with the Q, set the passive torque to the same index as its associated dof
    size_t idx(other.index());

    // If there are less actuators declared than dof, the vector must be enlarged
    if (idx >= m_pas->size()) {
        m_pas->resize(idx+1);
        m_isDofSet->resize(idx+1, false);
    }
    // Add a passive torque to the pool of passive torques according to its type
    if (other.type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_CONSTANT) {
        (*m_pas)[idx] = std::make_shared<internal_forces::passive_torques::PassiveTorqueConstant>
                (static_cast<const internal_forces::passive_torques::PassiveTorqueConstant&>(other));
        (*m_isDofSet)[idx] = true;
        return;
    } else if (other.type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_LINEAR) {
        (*m_pas)[idx] = std::make_shared<internal_forces::passive_torques::PassiveTorqueLinear>
                (static_cast<const internal_forces::passive_torques::PassiveTorqueLinear&>(other));
        (*m_isDofSet)[idx] = true;
        return;
    } else if (other.type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_EXPONENTIAL) {
        (*m_pas)[idx] = std::make_shared<internal_forces::passive_torques::PassiveTorqueExponential>
                (static_cast<const internal_forces::passive_torques::PassiveTorqueExponential&>(other));
        (*m_isDofSet)[idx] = true;
        return;
    } else {
        utils::Error::raise("Passive Torque type not found");
    }
    return;
}

const std::shared_ptr<internal_forces::passive_torques::PassiveTorque>&
      internal_forces::passive_torques::PassiveTorques::getPassiveTorque(size_t dof)
{
    utils::Error::check(dof<nbPassiveTorques(), "Idx asked is higher than number of passive torque");
    return (*m_pas)[dof];
}

size_t internal_forces::passive_torques::PassiveTorques::nbPassiveTorques() const
{
    return m_pas->size();
}

rigidbody::GeneralizedTorque internal_forces::passive_torques::PassiveTorques::passiveJointTorque(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
    rigidbody::GeneralizedTorque GeneralizedTorque_all = rigidbody::GeneralizedTorque(model);

    // If there are less actuators declared than dof, the vector must be enlarged
    if (model.nbDof() >= m_pas->size()) {
        m_pas->resize(model.nbDof() + 1);
        m_isDofSet->resize(model.nbDof() + 1, false);
    }

    for (unsigned int i=0; i<model.nbDof(); ++i) {
        if (!(*m_isDofSet)[i]) {
            GeneralizedTorque_all[i] = 0;
        } else if (std::dynamic_pointer_cast<PassiveTorqueConstant> ((*m_pas)[i])) {
            GeneralizedTorque_all[i] = std::static_pointer_cast<PassiveTorqueConstant>((*m_pas)[i])->passiveTorque();
        } else if (std::dynamic_pointer_cast<PassiveTorqueLinear> ((*m_pas)[i])) {
            GeneralizedTorque_all[i] = std::static_pointer_cast<PassiveTorqueLinear>((*m_pas)[i])->passiveTorque(Q);
        } else if (std::dynamic_pointer_cast<PassiveTorqueExponential> ((*m_pas)[i])) {
            GeneralizedTorque_all[i] = std::static_pointer_cast<PassiveTorqueExponential>((*m_pas)[i])->passiveTorque(Q, Qdot);
        } else {
            utils::Error::raise("Wrong type (should never get here because of previous safety)");
        }
    }
    return GeneralizedTorque_all;
}

