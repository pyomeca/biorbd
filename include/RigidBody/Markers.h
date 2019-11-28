#ifndef BIORBD_RIGIDBODY_MARKERS_H
#define BIORBD_RIGIDBODY_MARKERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Matrix;
}

namespace rigidbody {
class GeneralizedCoordinates;
class NodeSegment;

///
/// \brief Class Markers
///
class BIORBD_API Markers
{
public:
    ///
    /// \brief Construct markers
    ///
    Markers();
    ///
    /// \brief Construct markers from other markers
    /// \param other The other markers 
    ///
    Markers(const biorbd::rigidbody::Markers& other);
    ///
    /// \brief Destroy class properly
    ///
    virtual ~Markers();
    ///
    /// \brief Deep copy of the markers
    /// \return Deep copy of the markers
    ///
    biorbd::rigidbody::Markers DeepCopy() const;

    /// 
    /// \brief Deep copy of the markers 
    /// \param other The markers to copy from
    /// 
    void DeepCopy(const biorbd::rigidbody::Markers& other);

    // Set and get

    /// 
    /// \brief Add a marker
    /// \param pos The position of the marker
    /// \param name The name of the marker
    /// \param parentName The name of the marker's parent
    /// \param technical If the marker is technical
    /// \param anatomical If the marker is anatomical
    /// \param axesToRemove Axes to remove
    /// \param id The marker identification (default: -1)
    ///
    void addMarker(
            const biorbd::rigidbody::NodeSegment &pos,
            const biorbd::utils::String &name,
            const biorbd::utils::String &parentName,
            bool technical,
            bool anatomical,
            const biorbd::utils::String& axesToRemove,
            int id = -1
        );

    ///
    /// \brief Return marker i
    /// \param i The marker we want to return
    /// \return The marker
    ///
    const biorbd::rigidbody::NodeSegment& marker(
            unsigned int i) const;

    ///
    /// \brief Return the markers on a segment i
    /// \param segmentName Name of the segment i
    /// \return The markers on segment i
    ///
    std::vector<biorbd::rigidbody::NodeSegment> marker(
            const biorbd::utils::String &segmentName) const;
    ///
    /// \brief Return the names of the markers
    /// \return The names of the markers
    ///
    std::vector<biorbd::utils::String> markerNames() const;
    
    ///
    /// \brief Return the names of the technical markers
    /// \return The names of the technical markers
    ///
    std::vector<biorbd::utils::String> technicalMarkerNames() const;

    ///
    /// \brief Return the names of the anatomical markers
    /// \return The names of the anatomical markers
    ///
    std::vector<biorbd::utils::String> anatomicalMarkerNames() const;

    ///
    /// \brief Return a marker
    /// \param Q The generalized coordinates 
    /// \param node The node on which the marker is
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return A marker
    ///
    biorbd::rigidbody::NodeSegment marker(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::NodeSegment& node,
            bool removeAxis=true,
            bool updateKin = true);

    ///
    /// \brief Return a marker
    /// \param Q The generalized coordinates 
    /// \param idx The marker identification
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return A marker
    ///
    biorbd::rigidbody::NodeSegment marker(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); 

    ///
    /// \brief Return a marker
    /// \param idx The marker identification
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \return A marker
    ///
    biorbd::rigidbody::NodeSegment marker(
            unsigned int  idx,
            bool removeAxis);

    ///
    /// \brief Return all the markers
    /// \param Q The generalized coordinates of the model
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return A vector of all the markers
    ///
    std::vector<biorbd::rigidbody::NodeSegment> markers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); 

    ///
    /// \brief Return all the markers
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \return A vector of all the markers
    ///
    std::vector<biorbd::rigidbody::NodeSegment> markers(
            bool removeAxis=true); 

    ///
    /// \brief Return the velocity of a marker
    /// \param Q The generalized coordinates of the model
    /// \param Qdot The generalized velocities of the model
    /// \param idx The marker identification
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return The velocity of a marker
    ///
    biorbd::rigidbody::NodeSegment markerVelocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true);


    ///
    /// \brief Return the velocity of all markers
    /// \param Q The generalized coordinates of the model
    /// \param Qdot The generalized velocities of the model
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return A vector of the velocity of all markers
    ///
    std::vector<biorbd::rigidbody::NodeSegment> markersVelocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool removeAxis=true,
            bool updateKin = true); 

    ///
    /// \brief Return the technical markers
    /// \param Q The generalized coordinates of the model
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return A vector of all the technical markers
    ///
    std::vector<biorbd::rigidbody::NodeSegment> technicalMarkers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); 
    ///
    /// \brief Return the technical markers
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \return A vector of all the technical markers
    ///
    std::vector<biorbd::rigidbody::NodeSegment> technicalMarkers(
            bool removeAxis=true);

    ///
    /// \brief Return the anatomical markers
    /// \param Q The generalized coordinates of the model
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return A vector of all the anatomical markers
    ///
    std::vector<biorbd::rigidbody::NodeSegment> anatomicalMarkers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); 

    ///
    /// \brief Return the anatomical markers
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \return A vector of all the technical markers
    ///
    std::vector<biorbd::rigidbody::NodeSegment> anatomicalMarkers(
            bool removeAxis=true); 


    ///
    /// \brief Return the markers of segment i
    /// \param Q The generalized coordinates
    /// \param idx The segment identification
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return A vector of the markers of segment i
    ///
    std::vector<biorbd::rigidbody::NodeSegment> segmentMarkers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); 


    /// 
    /// \brief Return the number of markers
    /// \return The number of markers
    ///
    unsigned int nbMarkers() const;

    ///
    /// \brief Return the number of markers of segment i. If no segment input, then it returns the total number of markers
    /// \param idxSegment The segment identification
    /// \return The number of markers of segment i
    ///
    unsigned int nbMarkers(unsigned int idxSegment) const; 

    /// 
    /// \brief Return the number of technical markers
    /// \return The number of technical markers
    ///
    unsigned int nbTechnicalMarkers();

    /// 
    /// \brief Return the number of technical markers of segment i
    /// \param idxSegment The segment identification
    /// \return The number of technical markers of segment i
    ///
    unsigned int nbTechnicalMarkers(unsigned int idxSegment); 

    ///
    /// \brief Return the number of anatomical markers
    /// \return The number of anatomical markers
    ///
    unsigned int nbAnatomicalMarkers(); 

    ///
    /// \brief Return the Jacobian of the markers
    /// \param Q The generalized coordinates of the model
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return The Jacobian of the markers
    ///
    std::vector<biorbd::utils::Matrix> markersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true);

    /// 
    /// \brief Return the Jacobian of the technical markers
    /// \param Q The position variable of the model
    /// \param removeAxis If there are axis to remove from the position variables (default: True) TODO
    /// \param updateKin If the kinematics should be updated (default: True)
    /// \return The Jacobian of the technical markers
    ///
    std::vector<biorbd::utils::Matrix> TechnicalMarkersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true);

    ///
    /// \brief Return the Jacobian of a chosen marker
    /// \param Q The generalized coordinates of the model
    /// \param parentName The marker's parent name
    /// \param p  The position of the point in body-local data
    /// \param  updateKin If the kinematics should be updated
    /// \return The Jacobian of a chosen marker
    ///
    biorbd::utils::Matrix markersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::String& parentName,
            const biorbd::rigidbody::NodeSegment& p,
            bool updateKin); 

protected:
    ///
    /// \brief Return the Jacobian of the markers
    /// \param Q The generalized coordinates of the model
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the kinematics should be updated
    /// \param lookForTechnical Check if there are technical markers
    /// \return The Jacobian of the markers
    ///
    std::vector<biorbd::utils::Matrix> markersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis,
            bool updateKin,
            bool lookForTechnical); // Retourne la jacobienne des markers

    std::shared_ptr<std::vector<biorbd::rigidbody::NodeSegment>> m_marks; ///< The markers

};

}}

#endif // BIORBD_RIGIDBODY_MARKERS_H
