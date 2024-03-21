#define BIORBD_API_EXPORTS
#include "RigidBody/IMUs.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "Utils/Rotation.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Segment.h"
#include "RigidBody/IMU.h"

using namespace BIORBD_NAMESPACE;

rigidbody::IMUs::IMUs() :
    m_IMUs(std::make_shared<std::vector<rigidbody::IMU>>())
{
    //ctor
}

rigidbody::IMUs::IMUs(const rigidbody::IMUs &other)
{
    m_IMUs = other.m_IMUs;
}

rigidbody::IMUs::~IMUs()
{

}

rigidbody::IMUs rigidbody::IMUs::DeepCopy() const
{
    rigidbody::IMUs copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::IMUs::DeepCopy(const rigidbody::IMUs &other)
{
    m_IMUs->resize(other.m_IMUs->size());
    for (size_t i=0; i<other.m_IMUs->size(); ++i) {
        (*m_IMUs)[i] = (*other.m_IMUs)[i].DeepCopy();
    }
}

void rigidbody::IMUs::addIMU(
    bool technical,
    bool anatomical)
{
    m_IMUs->push_back(rigidbody::IMU(technical, anatomical));
}

// Add a new marker to the existing pool of markers
void rigidbody::IMUs::addIMU(
    const utils::RotoTransNode &RotoTrans,
    bool technical,
    bool anatomical)
{
    m_IMUs->push_back(rigidbody::IMU(RotoTrans, technical, anatomical));
}

size_t rigidbody::IMUs::nbIMUs() const
{
    return m_IMUs->size();
}


// Get the markers in the global reference
const std::vector<rigidbody::IMU>& rigidbody::IMUs::IMU() const
{
    return *m_IMUs;
}

std::vector<rigidbody::IMU> rigidbody::IMUs::IMU(
    const utils::String& segmentName)
{
    std::vector<rigidbody::IMU> pos;
    for (size_t i=0; i<nbIMUs();
            ++i) // Scan through all the markers and select the good ones
        if (!IMU(i).parent().compare(segmentName)) {
            pos.push_back(IMU(i));
        }
    return pos;
}

const rigidbody::IMU& rigidbody::IMUs::IMU(
    size_t idx)
{
    return (*m_IMUs)[idx];
}

// Get the IMUs at the position given by Q
std::vector<rigidbody::IMU> rigidbody::IMUs::IMU(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::IMU> pos;
    for (size_t i=0; i<nbIMUs(); ++i) {
        pos.push_back(IMU(Q, i, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get an IMU at the position given by Q
rigidbody::IMU rigidbody::IMUs::IMU(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    rigidbody::IMU node = IMU(idx);
    return model.globalJCS(Q, node.parent(), updateKin) * node;
}

// Get the technical IMUs
std::vector<rigidbody::IMU> rigidbody::IMUs::technicalIMU(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::IMU> pos;
    for (size_t i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isTechnical() ) {
            pos.push_back(IMU(Q, i, updateKin));
            updateKin = false;
        }
    return pos;
}
// Get the technical IMUs in the local reference
std::vector<rigidbody::IMU> rigidbody::IMUs::technicalIMU()
{
    std::vector<rigidbody::IMU> pos;
    for (size_t i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isTechnical() ) {
            pos.push_back(IMU(i));    // Forward kinematics
        }
    return pos;
}

// Get all the anatomical IMUs
std::vector<rigidbody::IMU> rigidbody::IMUs::anatomicalIMU(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::IMU> pos;
    for (size_t i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isAnatomical() ) {
            pos.push_back(IMU(Q, i, updateKin));
            updateKin = false;
        }
    return pos;
}
// Get the anatomical IMUs in the local reference
std::vector<rigidbody::IMU> rigidbody::IMUs::anatomicalIMU()
{
    std::vector<rigidbody::IMU> pos;
    for (size_t i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isAnatomical() ) {
            pos.push_back(IMU(i));    // Forward kinematics
        }
    return pos;
}

std::vector<rigidbody::IMU> rigidbody::IMUs::segmentIMU(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);

    // Segment name to find
    utils::String name(model.segment(idx).name());

    std::vector<rigidbody::IMU> pos;
    for (size_t i=0; i<nbIMUs();
            ++i) // scan all the markers and select the right ones
        if (!((*m_IMUs)[i]).parent().compare(name)) {
            pos.push_back(IMU(Q,i,updateKin));
            updateKin = false;
        }

    return pos;
}

// Get the Jacobian of the markers
std::vector<utils::Matrix> rigidbody::IMUs::IMUJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return IMUJacobian(Q, updateKin, false);
}

// Get the Jacobian of the technical markers
std::vector<utils::Matrix>
rigidbody::IMUs::TechnicalIMUJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return IMUJacobian(Q, updateKin, true);
}


// Protected function
std::vector<utils::Matrix> rigidbody::IMUs::IMUJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin,
    bool lookForTechnical)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);

    std::vector<utils::Matrix> G;

    for (size_t idx=0; idx<nbIMUs(); ++idx) {
        // Actual marker
        rigidbody::IMU node = IMU(idx);
        if (lookForTechnical && !node.isTechnical()) {
            continue;
        }

        size_t id = static_cast<size_t>(model.GetBodyId(node.parent().c_str()));
        utils::Matrix G_tp(utils::Matrix::Zero(9,model.dof_count));

        // Calculate the Jacobian of this Tag
        model.CalcMatRotJacobian(Q, id, node.rot(), G_tp, updateKin); 
#ifndef BIORBD_USE_CASADI_MATH
        updateKin = false;
#endif

        G.push_back(G_tp);
    }

    return G;
}

size_t rigidbody::IMUs::nbTechIMUs()
{
    size_t nbTech = 0;
    if (nbTech == 0) // If the function has never been called before
        for (rigidbody::IMU imu : *m_IMUs)
            if (imu.isTechnical()) {
                ++nbTech;
            }

    return nbTech;
}

size_t rigidbody::IMUs::nbAnatIMUs()
{
    size_t nbAnat = 0;
    if (nbAnat == 0) // If the function has never been called before
        for (rigidbody::IMU imu : *m_IMUs)
            if (imu.isAnatomical()) {
                ++nbAnat;
            }

    return nbAnat;
}

std::vector<utils::String> rigidbody::IMUs::IMUsNames()
{
    // Extract the name of all the markers of a model
    std::vector<utils::String> names;
    for (size_t i=0; i<nbIMUs(); ++i) {
        names.push_back(IMU(i).utils::Node::name());
    }
    return names;
}

std::vector<utils::String> rigidbody::IMUs::technicalIMUsNames()
{
    // Extract the names of all the technical markers of a model
    std::vector<utils::String> names;
    for (size_t i=0; i<nbIMUs(); ++i)
        if (IMU(i).isTechnical()) {
            names.push_back(IMU(i).utils::Node::name());
        }

    return names;
}

std::vector<utils::String>
rigidbody::IMUs::anatomicalIMUsNames()
{
    // Extract the names of all the anatomical markers of a model
    std::vector<utils::String> names;
    for (size_t i=0; i<nbIMUs(); ++i)
        if (IMU(i).isAnatomical()) {
            names.push_back(IMU(i).utils::Node::name());
        }

    return names;
}
