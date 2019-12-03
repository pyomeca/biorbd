#define BIORBD_API_EXPORTS
#include "Muscles/Muscles.h"

#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "Muscles/Muscle.h"
#include "Muscles/Geometry.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/Force.h"

biorbd::muscles::Muscles::Muscles() :
    m_mus(std::make_shared<std::vector<biorbd::muscles::MuscleGroup>>())
{

}

biorbd::muscles::Muscles::Muscles(const biorbd::muscles::Muscles &other) :
    m_mus(other.m_mus)
{

}

biorbd::muscles::Muscles::~Muscles()
{

}

biorbd::muscles::Muscles biorbd::muscles::Muscles::DeepCopy() const
{
    biorbd::muscles::Muscles copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::Muscles::DeepCopy(const biorbd::muscles::Muscles &other)
{
    m_mus->resize(other.m_mus->size());
    for (unsigned int i=0; i<other.m_mus->size(); ++i)
        (*m_mus)[i] = (*other.m_mus)[i];
}


void biorbd::muscles::Muscles::addMuscleGroup(
        const biorbd::utils::String &name,
        const biorbd::utils::String &originName,
        const biorbd::utils::String &insertionName)
{
    if (m_mus->size() > 0)
        biorbd::utils::Error::check(getGroupId(name)==-1, "Muscle group already defined");

    m_mus->push_back(biorbd::muscles::MuscleGroup(name, originName, insertionName));
}

int biorbd::muscles::Muscles::getGroupId(const biorbd::utils::String &name) const{
    for (unsigned int i=0; i<m_mus->size(); ++i)
        if (!name.compare((*m_mus)[i].name()))
            return static_cast<int>(i);
    return -1;
}

biorbd::muscles::MuscleGroup &biorbd::muscles::Muscles::muscleGroup(unsigned int idx)
{
    biorbd::utils::Error::check(idx<nbMuscleGroups(), "Idx asked is higher than number of muscle groups");
    return (*m_mus)[idx];
}

const biorbd::muscles::MuscleGroup &biorbd::muscles::Muscles::muscleGroup(unsigned int idx) const{
    biorbd::utils::Error::check(idx<nbMuscleGroups(), "Idx asked is higher than number of muscle groups");
    return (*m_mus)[idx];
}
const biorbd::muscles::MuscleGroup &biorbd::muscles::Muscles::muscleGroup(const biorbd::utils::String& name) const{
    int idx = getGroupId(name);
    biorbd::utils::Error::check(idx!=-1, "Group name could not be found");
    return muscleGroup(static_cast<unsigned int>(idx));
}

// From muscle activation (return muscle force)
biorbd::rigidbody::GeneralizedTorque biorbd::muscles::Muscles::muscularJointTorque(
        const std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> &emg,
        biorbd::utils::Vector &F,
        bool updateKin,
        const biorbd::rigidbody::GeneralizedCoordinates* Q,
        const biorbd::rigidbody::GeneralizedCoordinates* QDot)
{
    // Update the muscular position
    if (updateKin)
        updateMuscles(*Q,*QDot,updateKin);

    const std::vector<std::vector<std::shared_ptr<biorbd::muscles::Force>>>& force_tp = musclesForces(emg, false);
    F = biorbd::utils::Vector(static_cast<unsigned int>(force_tp.size()));
    for (unsigned int i=0; i<force_tp.size(); ++i)
        F(i) = (force_tp[i])[0]->norm();

    return muscularJointTorque(F, false, Q, QDot);
}

// From muscle activation (do not return muscle force)
biorbd::rigidbody::GeneralizedTorque biorbd::muscles::Muscles::muscularJointTorque(
        const std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>>& emg,
        bool updateKin,
        const biorbd::rigidbody::GeneralizedCoordinates* Q,
        const biorbd::rigidbody::GeneralizedCoordinates* QDot)
{
    biorbd::rigidbody::GeneralizedCoordinates dummy;
    return muscularJointTorque(emg, dummy, updateKin, Q, QDot);
}

// From Muscular Force
biorbd::rigidbody::GeneralizedTorque biorbd::muscles::Muscles::muscularJointTorque(
        const biorbd::utils::Vector &F,
        bool updateKin,
        const biorbd::rigidbody::GeneralizedCoordinates* Q,
        const biorbd::rigidbody::GeneralizedCoordinates* QDot)
{

    // Update the muscular position
    if (updateKin)
        updateMuscles(*Q,*QDot,updateKin);

    // Get the Jacobian matrix and get the forces of each muscle

    const biorbd::utils::Matrix& jaco(musclesLengthJacobian());

    // Compute the reaction of the forces on the bodies
    return biorbd::rigidbody::GeneralizedTorque( -jaco.transpose() * F );
}

std::vector<std::vector<std::shared_ptr<biorbd::muscles::Force>>> biorbd::muscles::Muscles::musclesForces(
        const std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> &emg,
        bool updateKin,
        const biorbd::rigidbody::GeneralizedCoordinates* Q,
        const biorbd::rigidbody::GeneralizedCoordinates* QDot)
{
    // Update the muscular position
    if (updateKin)
        updateMuscles(*Q,*QDot,updateKin);

    // Output variable
    std::vector<std::vector<std::shared_ptr<biorbd::muscles::Force>>> forces; // All the muscles/two pointers per muscleTous les muscles (origine/insertion)

    unsigned int cmpMus(0);
    for (unsigned int i=0; i<m_mus->size(); ++i) // muscle group
        for (unsigned int j=0; j<(*m_mus)[i].nbMuscles(); ++j)
            forces.push_back((*m_mus)[i].muscle(j).force(*emg[cmpMus++]));

    // The forces
    return forces;
}

unsigned int biorbd::muscles::Muscles::nbMuscleGroups() const {
    return static_cast<unsigned int>(m_mus->size());
}

biorbd::utils::Matrix biorbd::muscles::Muscles::musclesLengthJacobian()
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    biorbd::utils::Matrix tp(nbMuscleTotal(), model.nbDof());
    unsigned int cmpMus(0);
    for (unsigned int i=0; i<nbMuscleGroups(); ++i)
        for (unsigned int j=0; j<((*m_mus)[i]).nbMuscles(); ++j)
            tp.block(cmpMus++,0,1,model.nbDof()) = ((*m_mus)[i]).muscle(j).position().jacobianLength();

    return tp;

}

biorbd::utils::Matrix biorbd::muscles::Muscles::musclesLengthJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    // Update the muscular position
    updateMuscles(Q, true);
    return musclesLengthJacobian();
}


unsigned int biorbd::muscles::Muscles::nbMuscleTotal() const{
    unsigned int total(0);
    for (unsigned int grp=0; grp<m_mus->size(); ++grp) // muscular group
        total += (*m_mus)[grp].nbMuscles();
    return total;
}

void biorbd::muscles::Muscles::updateMuscles(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedCoordinates& QDot,
        bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    // Update all the muscles
    int updateKinTP;
    if (updateKin)
        updateKinTP = 2;
    else
        updateKinTP = 0;

    for (auto group : *m_mus) // muscle group
        for (unsigned int j=0; j<group.nbMuscles(); ++j){
            group.muscle(j).updateOrientations(model, Q, QDot, updateKinTP);
            updateKinTP=1;
        }
}
void biorbd::muscles::Muscles::updateMuscles(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    // Update all the muscles
    int updateKinTP;
    if (updateKin)
        updateKinTP = 2;
    else
        updateKinTP = 0;

    // Update all the muscles
    for (auto group : *m_mus) // muscle group
        for (unsigned int j=0; j<group.nbMuscles(); ++j){
            group.muscle(j).updateOrientations(model, Q,updateKinTP);
            updateKinTP=1;
        }
}
void biorbd::muscles::Muscles::updateMuscles(
        std::vector<std::vector<biorbd::utils::Vector3d>>& musclePointsInGlobal,
        std::vector<biorbd::utils::Matrix> &jacoPointsInGlobal,
        const biorbd::rigidbody::GeneralizedCoordinates& QDot)
{
    unsigned int cmpMuscle = 0;
    for (auto group : *m_mus) // muscle  group
        for (unsigned int j=0; j<group.nbMuscles(); ++j){
            group.muscle(j).updateOrientations(musclePointsInGlobal[cmpMuscle], jacoPointsInGlobal[cmpMuscle], QDot);
            ++cmpMuscle;
        }
}
void biorbd::muscles::Muscles::updateMuscles(
        std::vector<std::vector<biorbd::utils::Vector3d>>& musclePointsInGlobal,
        std::vector<biorbd::utils::Matrix> &jacoPointsInGlobal)
{
    // Updater all the muscles
    unsigned int cmpMuscle = 0;
    for (auto group : *m_mus) // muscle group
        for (unsigned int j=0; j<group.nbMuscles(); ++j){
            group.muscle(j).updateOrientations(musclePointsInGlobal[cmpMuscle], jacoPointsInGlobal[cmpMuscle]);
            ++cmpMuscle;
        }
}
