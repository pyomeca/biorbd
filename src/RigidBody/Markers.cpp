#define BIORBD_API_EXPORTS
#include "RigidBody/Markers.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/Joints.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"

using namespace BIORBD_NAMESPACE;

rigidbody::Markers::Markers() :
    m_marks(std::make_shared<std::vector<rigidbody::NodeSegment>>())
{
    //ctor
}

rigidbody::Markers::Markers(const rigidbody::Markers &other) :
    m_marks(other.m_marks)
{

}

rigidbody::Markers::~Markers()
{
    //dtor
}

rigidbody::Markers rigidbody::Markers::DeepCopy() const
{
    rigidbody::Markers copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::Markers::DeepCopy(
        const rigidbody::Markers &other)
{
    m_marks->resize(other.m_marks->size());
    for (unsigned int i=0; i<other.m_marks->size(); ++i) {
        (*m_marks)[i] = (*other.m_marks)[i].DeepCopy();
    }
}

// Add a new marker to the markers pool
void rigidbody::Markers::addMarker(
    const rigidbody::NodeSegment &pos,
    const utils::String &name,
    const utils::String &parentName,
    bool technical,
    bool anatomical,
    const utils::String& axesToRemove,
    int id)
{
    rigidbody::NodeSegment tp(pos, name, parentName, technical, anatomical,
                                      axesToRemove, id);
    m_marks->push_back(tp);
}

const rigidbody::NodeSegment &rigidbody::Markers::marker(
    unsigned int idx) const
{
    return (*m_marks)[idx];
}

std::vector<rigidbody::NodeSegment> rigidbody::Markers::marker(
    const utils::String& name) const
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers();
            ++i) // Go through all the markers and select the right ones
        if (!marker(i).parent().compare(name)) {
            pos.push_back(marker(i));
        }

    return pos;
}

// Return a marker
rigidbody::NodeSegment rigidbody::Markers::marker(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::NodeSegment &n,
    bool removeAxis,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    unsigned int id = model.GetBodyId(n.parent().c_str());
    if (removeAxis) {
        return rigidbody::NodeSegment(
                   RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, n.removeAxes(),
                           updateKin));
    } else {
        return rigidbody::NodeSegment(
                   RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, n, updateKin));
    }
}

// Get a marker
rigidbody::NodeSegment rigidbody::Markers::marker(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx,
    bool removeAxis,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::NodeSegment& node(marker(idx));
    unsigned int id = model.GetBodyId(node.parent().c_str());

    // Retrieve the position of the marker in the local reference
    const rigidbody::NodeSegment& pos = marker(idx, removeAxis);

    return rigidbody::NodeSegment(
               RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, pos, updateKin));
}

// Get a marker
rigidbody::NodeSegment rigidbody::Markers::marker(
    unsigned int idx,
    bool removeAxis)
{
    const rigidbody::NodeSegment& node(marker(idx));
    if (removeAxis) {
        return node.removeAxes();
    } else {
        return node;
    }
}

// Get all the markers
std::vector<rigidbody::NodeSegment> rigidbody::Markers::markers(
    const rigidbody::GeneralizedCoordinates &Q,
    bool removeAxis,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        pos.push_back(marker(Q, i, removeAxis, updateKin));
        updateKin = false;
    }

    return pos;
}
// Get all the markers in the local reference
std::vector<rigidbody::NodeSegment> rigidbody::Markers::markers(
    bool removeAxis)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        pos.push_back(marker(i, removeAxis));    // Forward kinematics
    }

    return pos;
}

// Get a marker's velocity
rigidbody::NodeSegment rigidbody::Markers::markerVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    unsigned int idx,
    bool removeAxis,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::NodeSegment& node(marker(idx));
    unsigned int id(model.GetBodyId(node.parent().c_str()));

    // Retrieve the position of the marker in the local reference
    const rigidbody::NodeSegment& pos(marker(idx, removeAxis));

    // Calculate the velocity of the point
    return rigidbody::NodeSegment(RigidBodyDynamics::CalcPointVelocity(
            model, Q, Qdot, id, pos, updateKin));
}

// Get a marker's velocity
rigidbody::NodeSegment rigidbody::Markers::markerAngularVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    unsigned int idx,
    bool removeAxis,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::NodeSegment& node(marker(idx));
    unsigned int id(model.GetBodyId(node.parent().c_str()));

    // Retrieve the position of the marker in the local reference
    const rigidbody::NodeSegment& pos(marker(idx, removeAxis));

    // Calculate the velocity of the point
    return rigidbody::NodeSegment(
                RigidBodyDynamics::CalcPointVelocity6D(model, Q, Qdot, id, pos, updateKin).block(0, 0, 3, 1)
            );
}

// Get the makers' velocities
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::markersVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool removeAxis,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        pos.push_back(markerVelocity(Q, Qdot, i, removeAxis, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get the makers' velocities
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::markersAngularVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool removeAxis,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        pos.push_back(markerAngularVelocity(Q, Qdot, i, removeAxis, updateKin));
        updateKin = false;
    }

    return pos;
}

rigidbody::NodeSegment rigidbody::Markers::markerAcceleration(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    unsigned int idx,
    bool removeAxis,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    const rigidbody::NodeSegment& node(marker(idx));
    unsigned int id(model.GetBodyId(node.parent().c_str()));

    // Retrieve the position of the marker in the local reference
    const rigidbody::NodeSegment& pos(marker(idx, removeAxis));

    // Calculate the acceleration of the point
    return rigidbody::NodeSegment(RigidBodyDynamics::CalcPointAcceleration(
            model, Q, Qdot, Qddot, id, pos,
            updateKin));
}

std::vector<rigidbody::NodeSegment>
rigidbody::Markers::markerAcceleration(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool removeAxis,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        pos.push_back(markerAcceleration(Q, Qdot, Qddot, i, removeAxis, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get the technical markers
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::technicalMarkers(
    const rigidbody::GeneralizedCoordinates &Q,
    bool removeAxis,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        if ( marker(i).isTechnical() ) {
            pos.push_back(marker(Q, i, removeAxis, updateKin));
            updateKin = false;
        }
    }
    return pos;
}
// Get the technical markers in a local reference
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::technicalMarkers(
    bool removeAxis)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i)
        if ( marker(i).isTechnical() ) {
            pos.push_back(marker(i, removeAxis));// Forward kinematics
        }
    return pos;
}
// Get the anatomical markers
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::anatomicalMarkers(
    const rigidbody::GeneralizedCoordinates &Q,
    bool removeAxis,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        if ( marker(i).isAnatomical() ) {
            pos.push_back(marker(Q, i, removeAxis, updateKin));
            updateKin = false;
        }
    }
    return pos;
}
// Get the anatomical markers in a local reference
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::anatomicalMarkers(
    bool removeAxis)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers(); ++i)
        if ( marker(i).isAnatomical() ) {
            pos.push_back(marker(i, removeAxis));    // Forward kinematics
        }
    return pos;
}



std::vector<rigidbody::NodeSegment>
rigidbody::Markers::segmentMarkers(
    const rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx,
    bool removeAxis,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);

    // Name of the segment to find
    const utils::String& name(model.segment(idx).name());

    std::vector<rigidbody::NodeSegment> pos;
    for (unsigned int i=0; i<nbMarkers();
            ++i) // Go through all the markers and select the right ones
        if (!(*m_marks)[i].parent().compare(name)) {
            pos.push_back(marker(Q,i,removeAxis,updateKin));
            updateKin = false;
        }

    return pos;
}


unsigned int rigidbody::Markers::nbMarkers() const
{
    return static_cast<unsigned int>(m_marks->size());
}

unsigned int rigidbody::Markers::nbMarkers(unsigned int idxSegment)
const
{
    // Assuming that this is also a joint type (via BiorbdModel)
    const rigidbody::Joints &model =
        dynamic_cast<const rigidbody::Joints &>(*this);

    // Name of the segment to find
    const utils::String& name(model.segment(idxSegment).name());

    unsigned int n = 0;
    for (unsigned int i=0; i<nbMarkers();
            ++i) // Go through all the markers and select the right ones
        if (!(*m_marks)[i].parent().compare(name)) {
            ++n;
        }

    return n;
}

// Get the Jacobian of the markers
std::vector<utils::Matrix> rigidbody::Markers::markersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool removeAxis,
    bool updateKin)
{
    return markersJacobian(Q, removeAxis, updateKin, false);
}

std::vector<utils::Matrix>
rigidbody::Markers::technicalMarkersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool removeAxis,
    bool updateKin)
{
    return markersJacobian(Q, removeAxis, updateKin, true);
}

// Get the Jacobian of the technical markers
utils::Matrix rigidbody::Markers::markersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    const utils::String& parentName,
    const rigidbody::NodeSegment& p,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    utils::Matrix G(utils::Matrix::Zero(3, model.nbQ()));;

    // Calculate the Jacobien of this Tag
    unsigned int id = model.GetBodyId(parentName.c_str());
    RigidBodyDynamics::CalcPointJacobian(model, Q, id, p, G, updateKin);

    return G;
}

#ifndef BIORBD_USE_CASADI_MATH
bool rigidbody::Markers::inverseKinematics(
    const std::vector<rigidbody::NodeSegment> &markers,
    const rigidbody::GeneralizedCoordinates &Qinit,
    rigidbody::GeneralizedCoordinates &Q,
    bool removeAxes)
{
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
    model.UpdateKinematicsCustom(&Q); // also assert for dimensions

    // Find the technical markers only (body_point)
    std::vector<rigidbody::NodeSegment> body_point(
        technicalMarkers(removeAxes));
    std::vector<RigidBodyDynamics::Math::Vector3d> body_pointEigen;
    for (unsigned int i=0; i<body_point.size(); ++i) {
        body_pointEigen.push_back(body_point[i]);
    }

    std::vector<RigidBodyDynamics::Math::Vector3d> markersInRbdl;
    for (unsigned int i = 0; i<markers.size(); ++i) {
        markersInRbdl.push_back(markers[i]);
    }

    // Associate the body number to each technical marker (body_id)
    std::vector<unsigned int> body_id;
    for (unsigned int i=0; i<body_point.size(); ++i) {
        body_id.push_back( static_cast<unsigned int>((*(body_point.begin()
                           +i)).parentId()) );
    }

    // Call the base function
    return RigidBodyDynamics::InverseKinematics(
               dynamic_cast<rigidbody::Joints &>(*this),
               Qinit, body_id, body_pointEigen, markersInRbdl, Q);
}
#endif

// Get the Jacobian of the technical markers
std::vector<utils::Matrix> rigidbody::Markers::markersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool removeAxis,
    bool updateKin,
    bool lookForTechnical)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif

    std::vector<utils::Matrix> G;

    for (unsigned int idx=0; idx<nbMarkers(); ++idx) {
        // Actual marker
        const rigidbody::NodeSegment& node(marker(idx));
        if (lookForTechnical && !node.isTechnical()) {
            continue;
        }

        unsigned int id = model.GetBodyId(node.parent().c_str());
        const utils::Vector3d& pos(marker(idx, removeAxis));
        utils::Matrix G_tp(utils::Matrix::Zero(3,model.nbQ()));

        // Calculate the Jacobian of this Tag
        RigidBodyDynamics::CalcPointJacobian(model, Q, id, pos, G_tp, updateKin);
#ifndef BIORBD_USE_CASADI_MATH
        updateKin = false;
#endif

        G.push_back(G_tp);
    }

    return G;
}

unsigned int rigidbody::Markers::nbTechnicalMarkers()
{
    unsigned int nTechMarkers = 0;
    if (nTechMarkers == 0) // If the function has never been called before
        for (auto mark : *m_marks)
            if (mark.isTechnical()) {
                ++nTechMarkers;
            }

    return nTechMarkers;
}

unsigned int rigidbody::Markers::nbTechnicalMarkers(
    unsigned int idxSegment)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);

    unsigned int nTechMarkers = 0;

    // Name of the segment to find
    const utils::String& name(model.segment(idxSegment).name());

    if (nTechMarkers == 0) // If the function has never been called before
        for (auto mark : *m_marks)
            if (mark.isTechnical() && !mark.parent().compare(name)) {
                ++nTechMarkers;
            }

    return nTechMarkers;
}


unsigned int rigidbody::Markers::nbAnatomicalMarkers()
{
    unsigned int nAnatMarkers = 0;
    if (nAnatMarkers == 0) // If the function has never been called before
        for (auto mark : *m_marks)
            if (mark.isAnatomical()) {
                ++nAnatMarkers;
            }

    return nAnatMarkers;
}

std::vector<utils::String> rigidbody::Markers::markerNames()
const
{
    // Extract the name of all the markers of a model
    std::vector<utils::String> names;
    for (unsigned int i=0; i<nbMarkers(); ++i) {
        names.push_back(marker(i).utils::Node::name());
    }

    return names;
}

std::vector<utils::String>
rigidbody::Markers::technicalMarkerNames() const
{
    // Extract the name of all the technical markers of a model
    std::vector<utils::String> names;
    for (unsigned int i=0; i<nbMarkers(); ++i)
        if (marker(i).isTechnical()) {
            names.push_back(marker(i).utils::Node::name());
        }

    return names;
}

std::vector<utils::String>
rigidbody::Markers::anatomicalMarkerNames() const
{
    // Extract the names of all the anatomical markers of a model
    std::vector<utils::String> names;
    for (unsigned int i=0; i<nbMarkers(); ++i)
        if (marker(i).isAnatomical()) {
            names.push_back(marker(i).utils::Node::name());
        }

    return names;
}
