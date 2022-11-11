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
#include "InternalForces/PassiveTorques/PassiveTorques.h"

using namespace BIORBD_NAMESPACE;

internal_forces::passive_torques::PassiveTorques::PassiveTorques() :
    m_pas(std::make_shared<std::vector<std::shared_ptr<internal_forces::passive_torques::PassiveTorque>>>()),
    m_isDofSet(std::make_shared<std::vector<bool>>(1)),
    m_isClose(std::make_shared<bool>(false))
{
    (*m_isDofSet)[0] = false;
}

internal_forces::passive_torques::PassiveTorques::PassiveTorques(
    const internal_forces::passive_torques::PassiveTorques& other) :
    m_pas(other.m_pas),
    m_isDofSet(other.m_isDofSet),
    m_isClose(other.m_isClose)
{

}

internal_forces::passive_torques::PassiveTorques::~PassiveTorques()
{

}

void internal_forces::passive_torques::PassiveTorques::DeepCopy(const internal_forces::passive_torques::PassiveTorques
        &other)
{
    m_pas->resize(other.m_pas->size());
    for (unsigned int i=0; i<other.m_pas->size(); ++i) {
        if ((*other.m_pas)[i]->type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_CONSTANT) {
            (*m_pas)[i] = std::make_shared<internal_forces::passive_torques::PassiveTorqueConstant>
                    (static_cast<const internal_forces::passive_torques::PassiveTorqueConstant&>(*(*other.m_pas)[i]));
            return;
        }else if ((*other.m_pas)[i]->type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_LINEAR) {
            (*m_pas)[i] = std::make_shared<internal_forces::passive_torques::PassiveTorqueLinear>
                    (static_cast<const internal_forces::passive_torques::PassiveTorqueLinear&>(*(*other.m_pas)[i]));
            return;
        }
    }
    *m_isClose = *other.m_isClose;
    *m_isDofSet = *other.m_isDofSet;
}


void internal_forces::passive_torques::PassiveTorques::addPassiveTorque(const internal_forces::passive_torques::PassiveTorque
        &tor)
{
    utils::Error::check(
        !*m_isClose, "You can't add passive torque after closing the model");

    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Verify that the target dof is associated to a dof that
    // already exists in the model
    utils::Error::check(
        tor.index()<model.nbDof(), "Sent index is out of dof range");

    // For speed purposes and coherence with the Q,
    // set the passive torque to the same index as its associated dof
    unsigned int idx(tor.index());

    // If there are less actuators declared than dof,
    // the vector must be enlarged
    if (idx >= m_pas->size()) {
        m_pas->resize(idx+1);
        m_isDofSet->resize(idx+1, false);
    }
    // Add a passive torque to the pool of passive torques according to its type
    if (tor.type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_CONSTANT) {
        (*m_pas)[idx] = std::make_shared<internal_forces::passive_torques::PassiveTorqueConstant>
                (static_cast<const internal_forces::passive_torques::PassiveTorqueConstant&>(tor));
        (*m_isDofSet)[idx] = true;
        return;
    }else if (tor.type() == internal_forces::passive_torques::TORQUE_TYPE::TORQUE_LINEAR) {
        (*m_pas)[idx] = std::make_shared<internal_forces::passive_torques::PassiveTorqueLinear>
                (static_cast<const internal_forces::passive_torques::PassiveTorqueLinear&>(tor));
        (*m_isDofSet)[idx] = true;
        return;
    } else {
        utils::Error::raise("Passive Torque type not found");
    }
    return;
}

const std::shared_ptr<internal_forces::passive_torques::PassiveTorque>&
      internal_forces::passive_torques::PassiveTorques::getPassiveTorque(unsigned int dof)
{
//    utils::Error::check(dof<nbPassiveTorques(),
//                                "Idx asked is higher than number of passive torque");
    return (*m_pas)[dof];
}

void internal_forces::passive_torques::PassiveTorques::closePassiveTorque()
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    *m_isClose = true;
}

unsigned int internal_forces::passive_torques::PassiveTorques::nbPassiveTorques() const
{
    return static_cast<unsigned int>(m_pas->size());
}

rigidbody::GeneralizedTorque internal_forces::passive_torques::PassiveTorques::passiveJointTorque(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
    rigidbody::GeneralizedTorque GeneralizedTorque_all = rigidbody::GeneralizedTorque(model);

    for (unsigned int i=0; i<model.nbDof(); ++i) {
        if ((*m_isDofSet)[i]==false) {
            GeneralizedTorque_all[i] = 0;
        } else if (std::dynamic_pointer_cast<PassiveTorqueConstant> ((*m_pas)[i])) {
            GeneralizedTorque_all[i] = std::static_pointer_cast<PassiveTorqueConstant>
                                                ((*m_pas)[i])->passiveTorque();
        } else if (std::dynamic_pointer_cast<PassiveTorqueLinear> ((*m_pas)[i])) {
            GeneralizedTorque_all[i] = std::static_pointer_cast<PassiveTorqueLinear>
                                                ((*m_pas)[i])->passiveTorque(Q);

        } else {
            utils::Error::raise("Wrong type (should never get here because of previous safety)");
        }
    }
    return GeneralizedTorque_all;
}

