#ifndef BIORBD_RIGIDBODY_MARKERS_H
#define BIORBD_RIGIDBODY_MARKERS_H

#include <vector>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace utils {
class Matrix;
}

namespace rigidbody {
class GeneralizedCoordinates;
class NodeBone;

class BIORBD_API Markers
{
public:
    Markers();
    virtual ~Markers();

    // Set and get
    void addMarker(
            const biorbd::rigidbody::NodeBone &pos,
            const biorbd::utils::String &name = "",
            const biorbd::utils::String &parentName = "",
            bool technical = true,
            bool anatomical = false,
            const biorbd::utils::String& axesToRemove = biorbd::utils::String(),
            int id = -1
        );
    const biorbd::rigidbody::NodeBone& marker(
            unsigned int i) const;
    std::vector<biorbd::rigidbody::NodeBone> marker(
            const biorbd::utils::String &segmentName) const;
    std::vector<biorbd::utils::String> markerNames() const;
    std::vector<biorbd::utils::String> technicalMarkerNames() const;
    std::vector<biorbd::utils::String> anatomicalMarkerNames() const;

    biorbd::rigidbody::NodeBone Tags(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::NodeBone& node,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    biorbd::rigidbody::NodeBone Tags(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    biorbd::rigidbody::NodeBone Tags(
            unsigned int  idx,
            bool removeAxis=true); // Retour d'un marqueur ind idx
    std::vector<biorbd::rigidbody::NodeBone> Tags(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> Tags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    biorbd::rigidbody::NodeBone TagsVelocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    std::vector<biorbd::rigidbody::NodeBone> TagsVelocity(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> technicalTags(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> technicalTags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> anatomicalTags(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> anatomicalTags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> TechnicalTagsInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs techniques dans leur body
    std::vector<biorbd::rigidbody::NodeBone> AnatomicalTagsInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs anatomiques dans leur body
    std::vector<biorbd::rigidbody::NodeBone> segmentTags(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            unsigned int  idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs d'un segment


    unsigned int nTags() const; // Retourne le nombre de marqueurs
    unsigned int nTags(unsigned int idxSegment) const; // Retourne le nombre de marqueurs du segment idxSegment. Si aucun, somme de tous
    unsigned int nTechTags(); // Retourne le nombre de marqueurs techniques
    unsigned int nTechTags(unsigned int idxSegment); // Retourne le nombre de marqueurs techniques pour le segment idxSegment
    unsigned int nAnatTags(); // Retourne le nombre de marqueurs anatomiques
    std::vector<biorbd::utils::Matrix> TagsJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Tags
    std::vector<biorbd::utils::Matrix> TechnicalTagsJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Tags pour les marqueurs techniques
    biorbd::utils::Matrix TagsJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::utils::String& parentName,
            const biorbd::rigidbody::NodeBone& p,
            bool updateKin); // Jacobienne d'un marqueur au choix

protected:
    std::vector<biorbd::utils::Matrix> TagsJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool removeAxis,
            bool updateKin,
            bool lookForTechnical); // Retourne la jacobienne des Tags

    std::vector <biorbd::rigidbody::NodeBone> m_marks;

};

}}

#endif // BIORBD_RIGIDBODY_MARKERS_H
