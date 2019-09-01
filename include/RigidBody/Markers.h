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
class NodeBone;

class BIORBD_API Markers
{
public:
    Markers();
    Markers(const biorbd::rigidbody::Markers& other);
    virtual ~Markers();
    biorbd::rigidbody::Markers DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::Markers& other);

    // Set and get
    void addMarker(
            const biorbd::rigidbody::NodeBone &pos,
            const biorbd::utils::String &name,
            const biorbd::utils::String &parentName,
            bool technical,
            bool anatomical,
            const biorbd::utils::String& axesToRemove,
            int id = -1
        );
    const biorbd::rigidbody::NodeBone& marker(
            unsigned int i) const;
    std::vector<biorbd::rigidbody::NodeBone> marker(
            const biorbd::utils::String &segmentName) const;
    std::vector<biorbd::utils::String> markerNames() const;
    std::vector<biorbd::utils::String> technicalMarkerNames() const;
    std::vector<biorbd::utils::String> anatomicalMarkerNames() const;

    biorbd::rigidbody::NodeBone marker(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::NodeBone& node,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    biorbd::rigidbody::NodeBone marker(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    biorbd::rigidbody::NodeBone marker(
            unsigned int  idx,
            bool removeAxis); // Retour d'un marqueur ind idx
    std::vector<biorbd::rigidbody::NodeBone> markers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> markers(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    biorbd::rigidbody::NodeBone markerVelocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    std::vector<biorbd::rigidbody::NodeBone> markerVelocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> technicalMarkers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> technicalMarkers(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> anatomicalMarkers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> anatomicalMarkers(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> TechnicalMarkersInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs techniques dans leur body
    std::vector<biorbd::rigidbody::NodeBone> AnatomicalMarkersInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs anatomiques dans leur body
    std::vector<biorbd::rigidbody::NodeBone> segmentMarkers(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs d'un segment


    unsigned int nMarkers() const; // Retourne le nombre de marqueurs
    unsigned int nMarkers(unsigned int idxSegment) const; // Retourne le nombre de marqueurs du segment idxSegment. Si aucun, somme de tous
    unsigned int nTechnicalMarkers(); // Retourne le nombre de marqueurs techniques
    unsigned int nTechnicalMarkers(unsigned int idxSegment); // Retourne le nombre de marqueurs techniques pour le segment idxSegment
    unsigned int nAnatomicalMarkers(); // Retourne le nombre de marqueurs anatomiques
    std::vector<biorbd::utils::Matrix> markersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Markers
    std::vector<biorbd::utils::Matrix> TechnicalMarkersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Markers pour les marqueurs techniques
    biorbd::utils::Matrix markersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::String& parentName,
            const biorbd::rigidbody::NodeBone& p,
            bool updateKin); // Jacobienne d'un marqueur au choix

protected:
    std::vector<biorbd::utils::Matrix> markersJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis,
            bool updateKin,
            bool lookForTechnical); // Retourne la jacobienne des markers

    std::shared_ptr<std::vector<biorbd::rigidbody::NodeBone>> m_marks;

};

}}

#endif // BIORBD_RIGIDBODY_MARKERS_H
