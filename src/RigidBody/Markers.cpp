#define BIORBD_API_EXPORTS
#include "RigidBody/Markers.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/String.h"
#include "Utils/SpatialVector.h"
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
    for (size_t i=0; i<other.m_marks->size(); ++i) {
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
    int id)
{
    rigidbody::NodeSegment tp(pos, name, parentName, technical, anatomical, id);
    m_marks->push_back(tp);
}


void rigidbody::Markers::setMarker(
    size_t index,
    const rigidbody::NodeSegment& pos) 
{
#ifndef SKIP_ASSERT
    utils::Error::check(index < m_marks->size(), utils::String("The index ") + index + " is larger than the number of markers (" + m_marks->size() + ")");
#endif // SKIP_ASSERT
    (*m_marks)[index].setValues(pos);
}

std::vector<rigidbody::NodeSegment> rigidbody::Markers::marker(
    const utils::String& name) const
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers();
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
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);
    return rigidbody::NodeSegment(model.CalcBodyToBaseCoordinates(Q, n.parent(), n, updateKin));
}

// Get a marker
rigidbody::NodeSegment rigidbody::Markers::marker(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Retrieve the position of the marker in the local reference
    const rigidbody::NodeSegment& node(marker(idx));
    return rigidbody::NodeSegment(model.CalcBodyToBaseCoordinates(Q, node.parent(), node, updateKin));
}

// Get a marker
const rigidbody::NodeSegment& rigidbody::Markers::marker(
    size_t idx) const
{
    return (*m_marks)[idx];
}

// Get all the markers
std::vector<rigidbody::NodeSegment> rigidbody::Markers::markers(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) {
        pos.push_back(marker(Q, i, updateKin));
        updateKin = false;
    }

    return pos;
}
// Get all the markers in the local reference
std::vector<rigidbody::NodeSegment> rigidbody::Markers::markers()
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) {
        pos.push_back(marker(i)); 
    }

    return pos;
}

// Get a marker's velocity
rigidbody::NodeSegment rigidbody::Markers::markerVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Calculate the velocity of the point
    const rigidbody::NodeSegment& node(marker(idx));
    return rigidbody::NodeSegment(
        model.CalcPointVelocity(Q, Qdot, node.parent(), node, updateKin));
}

// Get a marker's velocity
rigidbody::NodeSegment rigidbody::Markers::markerAngularVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Calculate the velocity of the point
    const rigidbody::NodeSegment& node(marker(idx));
    return model.CalcPointVelocity6D(Q, Qdot, node.parent(), node, updateKin).block(0, 0, 3, 1);
}

// Get the makers' velocities
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::markersVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) {
        pos.push_back(markerVelocity(Q, Qdot, i, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get the makers' velocities
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::markersAngularVelocity(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) {
        pos.push_back(markerAngularVelocity(Q, Qdot, i, updateKin));
        updateKin = false;
    }

    return pos;
}

rigidbody::NodeSegment rigidbody::Markers::markerAcceleration(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &> (*this);

    // Calculate the acceleration of the point
    const rigidbody::NodeSegment& node(marker(idx));
    return rigidbody::NodeSegment(
        model.CalcPointAcceleration(Q, Qdot, Qddot, node.parent(), node, updateKin)
    );
}

std::vector<rigidbody::NodeSegment>
rigidbody::Markers::markerAcceleration(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedAcceleration &Qddot,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) {
        pos.push_back(markerAcceleration(Q, Qdot, Qddot, i, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get the technical markers
std::vector<rigidbody::NodeSegment>
rigidbody::Markers::technicalMarkers(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) {
        if ( marker(i).isTechnical() ) {
            pos.push_back(marker(Q, i, updateKin));
            updateKin = false;
        }
    }
    return pos;
}

// Get the technical markers in a local reference
std::vector<rigidbody::NodeSegment> rigidbody::Markers::technicalMarkers() const
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i = 0; i < nbMarkers(); ++i) {
        if (marker(i).isTechnical()) pos.push_back(marker(i));
    }
        
    return pos;
}
// Get the anatomical markers
std::vector<rigidbody::NodeSegment> rigidbody::Markers::anatomicalMarkers(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) {
        if ( marker(i).isAnatomical() ) {
            pos.push_back(marker(Q, i, updateKin));
            updateKin = false;
        }
    }
    return pos;
}
// Get the anatomical markers in a local reference
std::vector<rigidbody::NodeSegment> rigidbody::Markers::anatomicalMarkers() const
{
    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i = 0; i < nbMarkers(); ++i) {
        if (marker(i).isAnatomical()) pos.push_back(marker(i));
    }
    return pos;
}



std::vector<rigidbody::NodeSegment>
rigidbody::Markers::segmentMarkers(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);

    // Name of the segment to find
    const utils::String& name(model.segment(idx).name());

    std::vector<rigidbody::NodeSegment> pos;
    for (size_t i=0; i<nbMarkers(); ++i) { // Go through all the markers and select the right ones
        if (!(*m_marks)[i].parent().compare(name)) {
            pos.push_back(marker(Q, i, updateKin));
            updateKin = false;
        }
    }


    return pos;
}


size_t rigidbody::Markers::nbMarkers() const
{
    return m_marks->size();
}

size_t rigidbody::Markers::nbMarkers(size_t idxSegment)
const
{
    // Assuming that this is also a joint type (via BiorbdModel)
    const rigidbody::Joints &model =
        dynamic_cast<const rigidbody::Joints &>(*this);

    // Name of the segment to find
    const utils::String& name(model.segment(idxSegment).name());

    size_t n = 0;
    for (size_t i=0; i<nbMarkers();
            ++i) // Go through all the markers and select the right ones
        if (!(*m_marks)[i].parent().compare(name)) {
            ++n;
        }

    return n;
}

// Get the Jacobian of the markers
std::vector<utils::Matrix> rigidbody::Markers::markersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return markersJacobian(Q, updateKin, false);
}

std::vector<utils::Matrix> rigidbody::Markers::technicalMarkersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return markersJacobian(Q, updateKin, true);
}

// Get the Jacobian of the technical markers
utils::Matrix rigidbody::Markers::markersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    const utils::String& parentName,
    const rigidbody::NodeSegment& p,
    bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Calculate the Jacobien of this Tag
    return model.CalcPointJacobian(Q, parentName, p, updateKin);
}

#ifndef BIORBD_USE_CASADI_MATH
bool rigidbody::Markers::inverseKinematics(
    const std::vector<rigidbody::NodeSegment> &markers,
    const rigidbody::GeneralizedCoordinates &Qinit,
    rigidbody::GeneralizedCoordinates &Q)
{
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &> (*this);
    model.UpdateKinematicsCustom(&Q); // also assert for dimensions of Q. Warning: this call would not work with casadi

    // Find the technical markers only (bodyPoint)
    std::vector<rigidbody::NodeSegment> bodyPoint(technicalMarkers());
    std::vector<RigidBodyDynamics::Math::Vector3d> bodyPointEigen;
    for (size_t i=0; i< bodyPoint.size(); ++i) {
        bodyPointEigen.push_back(bodyPoint[i]);
    }

    std::vector<RigidBodyDynamics::Math::Vector3d> markersInRbdl;
    for (size_t i = 0; i<markers.size(); ++i) {
        markersInRbdl.push_back(markers[i]);
    }

    // Associate the body number to each technical marker (bodyPoint)
    std::vector<unsigned int> bodyId;
    for (size_t i=0; i< bodyPoint.size(); ++i) {
        bodyId.push_back( static_cast<size_t>((*(bodyPoint.begin()+i)).parentId()) );
    }

    // Call the base function
    return RigidBodyDynamics::InverseKinematics(
        dynamic_cast<rigidbody::Joints &>(*this),
        Qinit, 
        bodyId, 
        bodyPointEigen, 
        markersInRbdl, 
        Q
    );
}
#endif

// Get the Jacobian of the technical markers
std::vector<utils::Matrix> rigidbody::Markers::markersJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin,
    bool lookForTechnical)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &> (*this);

    std::vector<utils::Matrix> G;

    for (size_t idx=0; idx<nbMarkers(); ++idx) {
        // Actual marker
        const rigidbody::NodeSegment& node(marker(idx));
        if (lookForTechnical && !node.isTechnical()) {
            continue;
        }

        // Calculate the Jacobian of this Tag
        G.push_back(model.CalcPointJacobian(Q, node.parent(), node, updateKin));
        updateKin = false;
    }

    return G;
}

size_t rigidbody::Markers::nbTechnicalMarkers()
{
    size_t nTechMarkers = 0;
    if (nTechMarkers == 0) // If the function has never been called before
        for (auto mark : *m_marks)
            if (mark.isTechnical()) {
                ++nTechMarkers;
            }

    return nTechMarkers;
}

size_t rigidbody::Markers::nbTechnicalMarkers(
    size_t idxSegment)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    size_t nTechMarkers = 0;

    // Name of the segment to find
    const utils::String& name(model.segment(idxSegment).name());

    if (nTechMarkers == 0) // If the function has never been called before
        for (auto mark : *m_marks)
            if (mark.isTechnical() && !mark.parent().compare(name)) {
                ++nTechMarkers;
            }

    return nTechMarkers;
}


size_t rigidbody::Markers::nbAnatomicalMarkers()
{
    size_t nAnatMarkers = 0;
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
    for (size_t i=0; i<nbMarkers(); ++i) {
        names.push_back(marker(i).utils::Node::name());
    }

    return names;
}

std::vector<utils::String>
rigidbody::Markers::technicalMarkerNames() const
{
    // Extract the name of all the technical markers of a model
    std::vector<utils::String> names;
    for (size_t i=0; i<nbMarkers(); ++i)
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
    for (size_t i=0; i<nbMarkers(); ++i)
        if (marker(i).isAnatomical()) {
            names.push_back(marker(i).utils::Node::name());
        }

    return names;
}
