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

using namespace biorbd::BIORBD_MATH_NAMESPACE;

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
    for (unsigned int i=0; i<other.m_IMUs->size(); ++i) {
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
    const biorbd::utils::RotoTransNode &RotoTrans,
    bool technical,
    bool anatomical)
{
    m_IMUs->push_back(rigidbody::IMU(RotoTrans, technical, anatomical));
}

unsigned int rigidbody::IMUs::nbIMUs() const
{
    return static_cast<unsigned int>(m_IMUs->size());
}


// Get the markers in the global reference
const std::vector<rigidbody::IMU>& rigidbody::IMUs::IMU() const
{
    return *m_IMUs;
}

std::vector<rigidbody::IMU> rigidbody::IMUs::IMU(
    const biorbd::utils::String& segmentName)
{
    std::vector<rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs();
            ++i) // Scan through all the markers and select the good ones
        if (!IMU(i).parent().compare(segmentName)) {
            pos.push_back(IMU(i));
        }
    return pos;
}

const rigidbody::IMU& rigidbody::IMUs::IMU(
    unsigned int idx)
{
    return (*m_IMUs)[idx];
}

// Get the IMUs at the position given by Q
std::vector<rigidbody::IMU> rigidbody::IMUs::IMU(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs(); ++i) {
        pos.push_back(IMU(Q, i, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get an IMU at the position given by Q
rigidbody::IMU rigidbody::IMUs::IMU(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &model = dynamic_cast<biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        model.UpdateKinematicsCustom (&Q);
    }

    rigidbody::IMU node = IMU(idx);
    unsigned int id = static_cast<unsigned int>(model.GetBodyBiorbdId(
                          node.parent()));

    return model.globalJCS(id) * node;
}

// Get the technical IMUs
std::vector<rigidbody::IMU> rigidbody::IMUs::technicalIMU(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs(); ++i)
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
    for (unsigned int i=0; i<nbIMUs(); ++i)
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
    for (unsigned int i=0; i<nbIMUs(); ++i)
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
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isAnatomical() ) {
            pos.push_back(IMU(i));    // Forward kinematics
        }
    return pos;
}

std::vector<rigidbody::IMU> rigidbody::IMUs::segmentIMU(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &model = dynamic_cast<biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &>
                                       (*this);

    // Segment name to find
    biorbd::utils::String name(model.segment(idx).name());

    std::vector<rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs();
            ++i) // scan all the markers and select the right ones
        if (!((*m_IMUs)[i]).parent().compare(name)) {
            pos.push_back(IMU(Q,i,updateKin));
            updateKin = false;
        }

    return pos;
}

// Get the Jacobian of the markers
std::vector<biorbd::utils::Matrix> rigidbody::IMUs::IMUJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return IMUJacobian(Q, updateKin, false);
}

// Get the Jacobian of the technical markers
std::vector<biorbd::utils::Matrix>
rigidbody::IMUs::TechnicalIMUJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return IMUJacobian(Q, updateKin, true);
}


// Protected function
std::vector<biorbd::utils::Matrix> rigidbody::IMUs::IMUJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin,
    bool lookForTechnical)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &model = dynamic_cast<biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &>
                                       (*this);

    std::vector<biorbd::utils::Matrix> G;

    for (unsigned int idx=0; idx<nbIMUs(); ++idx) {
        // Actual marker
        rigidbody::IMU node = IMU(idx);
        if (lookForTechnical && !node.isTechnical()) {
            continue;
        }

        unsigned int id = model.GetBodyId(node.parent().c_str());
        biorbd::utils::Matrix G_tp(biorbd::utils::Matrix::Zero(9,model.dof_count));

        // Calculate the Jacobian of this Tag
        model.CalcMatRotJacobian(Q, id, node.rot(), G_tp, updateKin); // False for speed

        G.push_back(G_tp);
    }

    return G;
}

unsigned int rigidbody::IMUs::nbTechIMUs()
{
    unsigned int nbTech = 0;
    if (nbTech == 0) // If the function has never been called before
        for (rigidbody::IMU imu : *m_IMUs)
            if (imu.isTechnical()) {
                ++nbTech;
            }

    return nbTech;
}

unsigned int rigidbody::IMUs::nbAnatIMUs()
{
    unsigned int nbAnat = 0;
    if (nbAnat == 0) // If the function has never been called before
        for (rigidbody::IMU imu : *m_IMUs)
            if (imu.isAnatomical()) {
                ++nbAnat;
            }

    return nbAnat;
}

std::vector<biorbd::utils::String> rigidbody::IMUs::IMUsNames()
{
    // Extract the name of all the markers of a model
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nbIMUs(); ++i) {
        names.push_back(IMU(i).biorbd::utils::Node::name());
    }
    return names;
}

std::vector<biorbd::utils::String> rigidbody::IMUs::technicalIMUsNames()
{
    // Extract the names of all the technical markers of a model
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if (IMU(i).isTechnical()) {
            names.push_back(IMU(i).biorbd::utils::Node::name());
        }

    return names;
}

std::vector<biorbd::utils::String>
rigidbody::IMUs::anatomicalIMUsNames()
{
    // Extract the names of all the anatomical markers of a model
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if (IMU(i).isAnatomical()) {
            names.push_back(IMU(i).biorbd::utils::Node::name());
        }

    return names;
}
