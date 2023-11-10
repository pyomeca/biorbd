#define BIORBD_API_EXPORTS

#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"
#include "InternalForces/Ligaments/Ligament.h"
#include "InternalForces/Ligaments/Ligaments.h"
#include "InternalForces/Ligaments/LigamentConstant.h"
#include "InternalForces/Ligaments/LigamentSpringLinear.h"
#include "InternalForces/Ligaments/LigamentSpringSecondOrder.h"

using namespace BIORBD_NAMESPACE;

internal_forces::ligaments::Ligaments::Ligaments() :
    m_ligaments(std::make_shared<std::vector<std::shared_ptr<internal_forces::ligaments::Ligament>>>())
{

}

internal_forces::ligaments::Ligaments::Ligaments(const internal_forces::ligaments::Ligaments &other) :
    m_ligaments(other.m_ligaments)
{

}


internal_forces::ligaments::Ligaments::~Ligaments()
{

}


internal_forces::ligaments::Ligaments internal_forces::ligaments::Ligaments::DeepCopy() const
{
    internal_forces::ligaments::Ligaments copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::ligaments::Ligaments::DeepCopy(
        const internal_forces::ligaments::Ligaments &other)
{
    m_ligaments->resize(other.m_ligaments->size());
    for (size_t i=0; i<other.m_ligaments->size(); ++i) {
        if ((*other.m_ligaments)[i]->type() == internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_CONSTANT) {
            (*m_ligaments)[i] = std::make_shared<internal_forces::ligaments::LigamentConstant>((*other.m_ligaments)[i]);
            return;
        } else if ((*other.m_ligaments)[i]->type() == internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_LINEAR) {
            (*m_ligaments)[i] = std::make_shared<internal_forces::ligaments::LigamentSpringLinear>((*other.m_ligaments)[i]);
            return;
        } else if ((*other.m_ligaments)[i]->type() == internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_SECOND_ORDER) {
            (*m_ligaments)[i] = std::make_shared<internal_forces::ligaments::LigamentSpringSecondOrder>((*other.m_ligaments)[i]);
            return;
        }
    }
    *m_ligaments = *other.m_ligaments;
}

internal_forces::ligaments::Ligament& internal_forces::ligaments::Ligaments::ligament(size_t idx)
{
    utils::Error::check(idx<nbLigaments(),
                                "Idx asked is higher than number of ligaments");
    return *(*m_ligaments)[idx];
}

const internal_forces::ligaments::Ligament& internal_forces::ligaments::Ligaments::ligament(size_t idx) const
{
    utils::Error::check(idx<nbLigaments(),
                                "Idx asked is higher than number of ligaments");
    return *(*m_ligaments)[idx];
}

void internal_forces::ligaments::Ligaments::addLigament(
        const internal_forces::ligaments::Ligament &ligamentTp)
{
    // Add a passive torque to the pool of passive torques according to its type
    if (ligamentTp.type() == internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_CONSTANT) {
        m_ligaments->push_back(std::make_shared<internal_forces::ligaments::LigamentConstant>(ligamentTp));
    } else if (ligamentTp.type() == internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_LINEAR) {
        m_ligaments->push_back(std::make_shared<internal_forces::ligaments::LigamentSpringLinear>(ligamentTp));
    } else if (ligamentTp.type() == internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_SECOND_ORDER) {
        m_ligaments->push_back(std::make_shared<internal_forces::ligaments::LigamentSpringSecondOrder>(ligamentTp));
    } else {
        utils::Error::raise("Ligament type not found");
    }
    return;
}

std::vector<utils::String> internal_forces::ligaments::Ligaments::ligamentNames() const
{
    std::vector<utils::String> names;
    for (size_t i=0; i<m_ligaments->size(); ++i)
    {
        names.push_back((*m_ligaments)[i]->name());
    }
    return names;
}

// From ligament Force
rigidbody::GeneralizedTorque
internal_forces::ligaments::Ligaments::ligamentsJointTorque(
    const utils::Vector &F)
{
    // Get the Jacobian matrix and get the forces of each muscle
    const utils::Matrix& jaco(ligamentsLengthJacobian());

    // Compute the reaction of the forces on the bodies
    return rigidbody::GeneralizedTorque( -jaco.transpose() * F );
}

// From ligament Force and kinematics
rigidbody::GeneralizedTorque
internal_forces::ligaments::Ligaments::ligamentsJointTorque(
    const utils::Vector &F,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot)
{

    // Update the ligament position
    updateLigaments(Q, QDot, true);
    return ligamentsJointTorque(F);
}

// From kinematics
rigidbody::GeneralizedTorque
internal_forces::ligaments::Ligaments::ligamentsJointTorque(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot)
{
    return ligamentsJointTorque(ligamentForces(Q, QDot));
}

utils::Vector internal_forces::ligaments::Ligaments::ligamentForces(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
    // Output variable
    utils::Vector forces(nbLigaments());
    for (size_t j=0; j<nbLigaments(); ++j) {
        forces(static_cast<unsigned int>(j)) = ((*m_ligaments)[j]->force(model,Q, QDot));
    }


    // The forces
    return forces;
}

utils::Vector internal_forces::ligaments::Ligaments::ligamentForces(
    const rigidbody::GeneralizedCoordinates& Q
        )
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
    // Output variable
    utils::Vector forces(nbLigaments());
    for (size_t j=0; j<nbLigaments(); ++j) {
        forces(static_cast<unsigned int>(j)) = ((*m_ligaments)[j]->force(model,Q));
    }

    // The forces
    return forces;
}

utils::Matrix internal_forces::ligaments::Ligaments::ligamentsLengthJacobian()
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
    utils::Matrix tp(nbLigaments(), model.nbDof());
    for (size_t j=0; j<nbLigaments(); ++j) {
        tp.block(static_cast<unsigned int>(j),0,1, static_cast<unsigned int>(model.nbDof())) =
            (*m_ligaments)[j]->position().jacobianLength();
    }
    return tp;
}

utils::Matrix internal_forces::ligaments::Ligaments::ligamentsLengthJacobian(
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Update the ligament position
    updateLigaments(Q, true);
    return ligamentsLengthJacobian();
}

size_t internal_forces::ligaments::Ligaments::nbLigaments() const
{
    return m_ligaments->size();
}

std::vector<std::shared_ptr<internal_forces::ligaments::Ligament>>&
        internal_forces::ligaments::Ligaments::ligaments()
{
    return *m_ligaments;
}

const std::vector<std::shared_ptr<internal_forces::ligaments::Ligament>>&
        internal_forces::ligaments::Ligaments::ligaments() const
{
    return *m_ligaments;
}

int internal_forces::ligaments::Ligaments::ligamentID(const utils::String&
        nameToFind)
{
    for (size_t i=0; i<m_ligaments->size(); ++i) {
        if (!nameToFind.compare( (*m_ligaments)[i]->name()) ) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void internal_forces::ligaments::Ligaments::updateLigaments(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Update all the muscles
#ifdef BIORBD_USE_CASADI_MATH
    int updateKinTP = 2;
#else
    int updateKinTP;
    if (updateKin) {
        updateKinTP = 2;
    } else {
        updateKinTP = 0;
    }
#endif

    for (size_t j=0; j<nbLigaments(); ++j) {
        ligament(j).updateOrientations(model, Q, QDot, updateKinTP);
#ifndef BIORBD_USE_CASADI_MATH
            if (updateKinTP){
                updateKinTP=1;
            }
#endif
        }
}

void internal_forces::ligaments::Ligaments::updateLigaments(
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &> (*this);

#ifdef BIORBD_USE_CASADI_MATH
    int updateKinTP = 2;
#else
    int updateKinTP;
    if (updateKin) {
        updateKinTP = 2;
    } else {
        updateKinTP = 0;
    }
#endif

    for (size_t j=0; j<nbLigaments(); ++j) {
            ligament(j).updateOrientations(model, Q, updateKinTP);
#ifndef BIORBD_USE_CASADI_MATH
            if (updateKinTP){
                updateKinTP=1;
            }
#endif
        }
}

void internal_forces::ligaments::Ligaments::updateLigaments(
    std::vector<std::vector<utils::Vector3d>>& ligamentPointsInGlobal,
    std::vector<utils::Matrix> &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity& QDot)
{
    for (size_t j=0; j<nbLigaments(); ++j) {
            ligament(j).updateOrientations(ligamentPointsInGlobal[j], jacoPointsInGlobal[j], QDot);
    }
}

void internal_forces::ligaments::Ligaments::updateLigaments(
    std::vector<std::vector<utils::Vector3d>>& ligamentPointsInGlobal,
    std::vector<utils::Matrix> &jacoPointsInGlobal)
{
    for (size_t j=0; j<nbLigaments(); ++j) {
            ligament(j).updateOrientations(ligamentPointsInGlobal[j], jacoPointsInGlobal[j]);
    }
}
