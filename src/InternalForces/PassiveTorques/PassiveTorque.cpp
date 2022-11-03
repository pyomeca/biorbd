#define BIORBD_API_EXPORTS
#include "InternalForces/PassiveTorques/PassiveTorque.h"

#include "Utils/Error.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;
internal_forces::passive_torques::PassiveTorque::PassiveTorque() :
    m_type(std::make_shared<internal_forces::passive_torques::TYPE>
           (internal_forces::passive_torques::TYPE::NO_TYPE)),
    m_dof_type(std::make_shared<internal_forces::passive_torques::DOF_TYPE>
           (internal_forces::passive_torques::DOF_TYPE::NO_DOF_TYPE)),
    m_direction(std::make_shared<int>(0)),
    m_jointName(std::make_shared<utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(-1))
{

}

internal_forces::passive_torques::PassiveTorque::PassiveTorque(
    const internal_forces::passive_torques::PassiveTorque &other) :
    m_type(other.m_type),
    m_dof_type(other.m_dof_type),
    m_direction(other.m_direction),
    m_jointName(other.m_jointName),
    m_dofIdx(other.m_dofIdx)
{

}

internal_forces::passive_torques::PassiveTorque::PassiveTorque(
    int direction,
    unsigned int dofIdx) :
    m_type(std::make_shared<internal_forces::passive_torques::TYPE>
           (internal_forces::passive_torques::TYPE::NO_TYPE)),
    m_dof_type(std::make_shared<internal_forces::passive_torques::DOF_TYPE>
           (internal_forces::passive_torques::DOF_TYPE::NO_DOF_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{

}

internal_forces::passive_torques::PassiveTorque::PassiveTorque(
    int direction,
    unsigned int dofIdx,
    const utils::String &jointName) :
    m_type(std::make_shared<internal_forces::passive_torques::TYPE>
           (internal_forces::passive_torques::TYPE::NO_TYPE)),
    m_dof_type(std::make_shared<internal_forces::passive_torques::DOF_TYPE>
           (internal_forces::passive_torques::DOF_TYPE::NO_DOF_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<utils::String>(jointName)),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{
    utils::Error::check(*m_direction==-1 || *m_direction==1,
                                "Direction should be -1 or 1");
}

internal_forces::passive_torques::PassiveTorque::~PassiveTorque()
{

}

void internal_forces::passive_torques::PassiveTorque::DeepCopy(const internal_forces::passive_torques::PassiveTorque
        &other)
{
    *m_type = *other.m_type;
    *m_dof_type = *other.m_dof_type;
    *m_direction = *other.m_direction;
    *m_jointName = *other.m_jointName;
    *m_dofIdx = *other.m_dofIdx;
}

unsigned int internal_forces::passive_torques::PassiveTorque::index() const
{
    return *m_dofIdx;
}

int internal_forces::passive_torques::PassiveTorque::direction() const
{
    return *m_direction;
}

internal_forces::passive_torques::TYPE internal_forces::passive_torques::PassiveTorque::type() const
{
    return *m_type;
}

internal_forces::passive_torques::DOF_TYPE internal_forces::passive_torques::PassiveTorque::dof_type() const
{
    return *m_dof_type;
}
