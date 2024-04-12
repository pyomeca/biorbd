#ifndef BIORBD_RIGIDBODY_MARKERS_H
#define BIORBD_RIGIDBODY_MARKERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
class Matrix;
class Vector3d;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedAcceleration;
class NodeSegment;
class Joints;

///
/// \brief Holder for the marker set
///
class BIORBD_API Markers
{
public:
    ///
    /// \brief Construct a marker set
    ///
    Markers();

    ///
    /// \brief Construct markers from another marker set
    /// \param other The other marker set
    ///
    Markers(const Markers& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Markers();

    ///
    /// \brief Deep copy of the markers
    /// \return Deep copy of the markers
    ///
    Markers DeepCopy() const;

    ///
    /// \brief Deep copy of the markers
    /// \param other The markers to copy from
    ///
    void DeepCopy(const Markers& other);

    ///
    /// \brief Add a marker to the set
    /// \param pos The position of the marker
    /// \param name The name of the marker
    /// \param parentName The name of the segment the marker is attached on
    /// \param technical If the marker is technical
    /// \param anatomical If the marker is anatomical
    /// \param axesToRemove Axes to remove while projecting the marker
    /// \param id The index of the parent segment
    ///
    void addMarker(
        const NodeSegment &pos,
        const utils::String &name,
        const utils::String &parentName,
        bool technical,
        bool anatomical,
        const utils::String& axesToRemove,
        int id = -1
    );

    /// 
    /// \brief Change the values of a specific marker without changing other parameters
    /// \param index The index of the marker to change. If biorbd is compiled with SKIP_ASSERT, then no check is performed to ensure index is within bounds
    /// \param pos The new position of the marker
    /// 
    void setMarker(
        size_t index,
        const NodeSegment &pos
    );

    ///
    /// \brief Return the names of all the markers
    /// \return The names of the markers
    ///
    std::vector<utils::String> markerNames() const;

    ///
    /// \brief Return the names of all the technical markers
    /// \return The names of the technical markers
    ///
    std::vector<utils::String> technicalMarkerNames() const;

    ///
    /// \brief Return the names of all the anatomical markers
    /// \return The names of the anatomical markers
    ///
    std::vector<utils::String> anatomicalMarkerNames() const;

    ///
    /// \brief Return a marker of index idx in the marker set
    /// \param idx The index of the marker
    /// \return The marker of index idx
    ///
    const NodeSegment& marker(
        size_t idx) const;

    ///
    /// \brief Return a marker of index idx in the marker set with the axes removed
    /// \param idx The index of the marker
    /// \return The marker of index idx
    ///
    NodeSegment markerAxesRemoved(
        size_t idx) const;

    ///
    /// \brief Compute and return the position of a marker at given Q in the global reference frame
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param node The position of the marker in its parent reference frame
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The marker in the global reference frame
    ///
    NodeSegment marker(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates& Q,
        const NodeSegment& node,
        bool removeAxis = true);

    ///
    /// \brief Compute and return the position of a marker at given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param node The position of the marker in its parent reference frame
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The marker in the global reference frame
    ///
    NodeSegment marker(
        const GeneralizedCoordinates& Q,
        const NodeSegment& node,
        bool updateKin = true,
        bool removeAxis = true);

    ///
    /// \brief Compute and return the position of the marker of index idx at given Q in the global reference frame
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The marker idx in the global reference frame
    ///
    NodeSegment marker(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates& Q,
        size_t idx,
        bool removeAxis = true);

    ///
    /// \brief Compute and return the position of the marker of index idx at given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param idx The index of the marker in the marker set
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The marker idx in the global reference frame
    ///
    NodeSegment marker(
        const GeneralizedCoordinates& Q,
        size_t idx,
        bool updateKin = true,
        bool removeAxis = true);

    ///
    /// \brief Return a copy of all the markers in their respective parent reference frame
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return All the markers
    ///
    std::vector<NodeSegment> markers(
        bool removeAxis = true) const;

    ///
    /// \brief Return the markers on a segment
    /// \param segmentName Name of the segment
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The markers on the segment
    ///
    std::vector<NodeSegment> markers(
        const utils::String &segmentName, 
        bool removeAxis = true) const;

    ///
    /// \brief Return all the markers at a given Q in the global reference frame
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return All the markers in the global reference frame
    ///
    std::vector<NodeSegment> markers(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        bool removeAxis = true);

    ///
    /// \brief Return all the markers at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return All the markers in the global reference frame
    ///
    std::vector<NodeSegment> markers(
        const GeneralizedCoordinates &Q,
        bool updateKin = true,
        bool removeAxis = true);

    ///
    /// \brief Return the linear velocity of a marker
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The linear velocity of a marker
    ///
    NodeSegment markerVelocity(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        size_t idx,
        bool removeAxis = true);

    ///
    /// \brief Return the linear velocity of a marker
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the marker set
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The linear velocity of a marker
    ///
    NodeSegment markerVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        size_t idx,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the angular velocity of a marker
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The angular velocity of a marker
    ///
    NodeSegment markerAngularVelocity(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        size_t idx,
        bool removeAxis = true);

    ///
    /// \brief Return the angular velocity of a marker
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the marker set
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The angular velocity of a marker
    ///
    NodeSegment markerAngularVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        size_t idx,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the linear velocity of all the markers
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The linear velocity of all the markers
    ///
    std::vector<NodeSegment> markersVelocity(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool removeAxis = true);

    ///
    /// \brief Return the linear velocity of all the markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The linear velocity of all the markers
    ///
    std::vector<NodeSegment> markersVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the angular velocity of all the markers
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The angular velocity of all the markers
    ///
    std::vector<NodeSegment> markersAngularVelocity(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool removeAxis = true);

    ///
    /// \brief Return the angular velocity of all the markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The angular velocity of all the markers
    ///
    std::vector<NodeSegment> markersAngularVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the acceleration of a marker
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The acceleration of a marker
    ///
    NodeSegment markerAcceleration(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        size_t idx,
        bool removeAxis = true);

    ///
    /// \brief Return the acceleration of a marker
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param idx The index of the marker in the marker set
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The acceleration of a marker
    ///
    NodeSegment markerAcceleration(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        size_t idx,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the acceleration of all the markers
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized velocities
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The acceleration of all the markers
    ///
    std::vector<NodeSegment> markerAcceleration(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &dQdot,
        bool removeAxis = true);

    ///
    /// \brief Return the acceleration of all the markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized velocities
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The acceleration of all the markers
    ///
    std::vector<NodeSegment> markerAcceleration(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &dQdot,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return all the technical markers at a given Q in the global reference frame
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return A vector of all the technical markers
    ///
    std::vector<NodeSegment> technicalMarkers(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        bool removeAxis = true);

    ///
    /// \brief Return all the technical markers at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return A vector of all the technical markers
    ///
    std::vector<NodeSegment> technicalMarkers(
        const GeneralizedCoordinates &Q,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the technical markers in their respective parent reference frame
    /// \return A vector of all the technical markers in their parent reference frame
    ///
    std::vector<NodeSegment> technicalMarkers() const;

    ///
    /// \brief Return the technical markers in their respective parent reference frame
    /// \return A vector of all the technical markers in their parent reference frame
    ///
    std::vector<NodeSegment> technicalMarkersAxesRemoved() const;

    ///
    /// \brief Return all the anatomical markers at a given Q in the global reference frame
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return A vector of all the anatomical markers
    ///
    std::vector<NodeSegment> anatomicalMarkers(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        bool removeAxis = true);

    ///
    /// \brief Return all the anatomical markers at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return A vector of all the anatomical markers
    ///
    std::vector<NodeSegment> anatomicalMarkers(
        const GeneralizedCoordinates &Q,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the anatomical markers in their respective parent reference frame
    /// \return A vector of all the anatomical markers in their parent reference frame
    ///
    std::vector<NodeSegment> anatomicalMarkers() const;

    ///
    /// \brief Return the anatomical markers in their respective parent reference frame
    /// \return A vector of all the anatomical markers in their parent reference frame
    ///
    std::vector<NodeSegment> anatomicalMarkersAxesRemoved() const;

    ///
    /// \brief Return all the markers of the segment idx at a given Q in the global reference frame
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return All the markers of the segment idx
    ///
    std::vector<NodeSegment> segmentMarkers(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        size_t idx,
        bool removeAxis = true);

    ///
    /// \brief Return all the markers of the segment idx at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return All the markers of the segment idx
    ///
    std::vector<NodeSegment> segmentMarkers(
        const GeneralizedCoordinates &Q,
        size_t idx,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the number of markers
    /// \return The number of markers
    ///
    size_t nbMarkers() const;

    ///
    /// \brief Return the number of markers on the segment idxSegment
    /// \param idxSegment The index of the segment
    /// \return The number of markers of segment idxSegment
    ///
    size_t nbMarkers(
        size_t idxSegment) const;

    ///
    /// \brief Return the number of technical markers
    /// \return The number of technical markers
    ///
    size_t nbTechnicalMarkers();

    ///
    /// \brief Return the number of technical markers on the segment idxSegment
    /// \param idxSegment The index of the segment
    /// \return The number of technical markers of segment idxSegment
    ///
    size_t nbTechnicalMarkers(size_t idxSegment);

    ///
    /// \brief Return the number of anatomical markers
    /// \return The number of anatomical markers
    ///
    size_t nbAnatomicalMarkers();

    ///
    /// \brief Return the jacobian of the markers
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The jacobian of the markers
    ///
    std::vector<utils::Matrix> markersJacobian(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        bool removeAxis = true);

    ///
    /// \brief Return the jacobian of the markers
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The jacobian of the markers
    ///
    std::vector<utils::Matrix> markersJacobian(
        const GeneralizedCoordinates &Q,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the jacobian of the technical markers
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The jacobian of the technical markers
    ///
    std::vector<utils::Matrix> technicalMarkersJacobian(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        bool removeAxis = true);

    ///
    /// \brief Return the jacobian of the technical markers
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The jacobian of the technical markers
    ///
    std::vector<utils::Matrix> technicalMarkersJacobian(
        const GeneralizedCoordinates &Q,
        bool updateKin = true, 
        bool removeAxis = true);

    ///
    /// \brief Return the jacobian of a chosen marker
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates of the model
    /// \param parentName The name of the segment the marker lies on
    /// \param p The position of the point in the parent reference frame
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The jacobian of the chosen marker
    ///
    utils::Matrix markersJacobian(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        const utils::String& parentName,
        const NodeSegment& p,
        bool removeAxis = true);

    ///
    /// \brief Return the jacobian of a chosen marker
    /// \param Q The generalized coordinates of the model
    /// \param parentName The name of the segment the marker lies on
    /// \param p The position of the point in the parent reference frame
    /// \param updateKin If the model should be updated
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The jacobian of the chosen marker
    ///
    utils::Matrix markersJacobian(
        const GeneralizedCoordinates &Q,
        const utils::String& parentName,
        const NodeSegment& p,
        bool updateKin = true, 
        bool removeAxis = true);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Performs an inverse kinematics
    /// \param markers The markers to track
    /// \param Qinit The initial guess for the generalized coordinates
    /// \param Q The generalized coordinates (output) that tracks the markers
    /// \param removeAxis If the axesToRemove should be removed
    /// \return If the problem converged
    ///
    bool inverseKinematics(
        const std::vector<NodeSegment>& markers,
        const GeneralizedCoordinates& Qinit,
        GeneralizedCoordinates &Q, 
        bool removeAxis = true);
#endif

protected:
    ///
    /// \brief Compute the jacobian of the markers
    /// \param updatedModel The joint model updated to the proper kinematics level
    /// \param Q The generalized coordinates
    /// \param lookForTechnical Check if only technical markers are to be computed
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The jacobian of the markers
    ///
    std::vector<utils::Matrix> markersJacobian(
        rigidbody::Joints& updatedModel,
        const GeneralizedCoordinates &Q,
        bool lookForTechnical, 
        bool removeAxis);

    std::shared_ptr<std::vector<NodeSegment>> m_marks; ///< The markers

};

}
}

#endif // BIORBD_RIGIDBODY_MARKERS_H
