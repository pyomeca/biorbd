#ifndef BIORBD_RIGIDBODY_MARKERS_H
#define BIORBD_RIGIDBODY_MARKERS_H

#include <vector>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd { namespace utils {
class Matrix;
class GenCoord;
}}

namespace biorbd { namespace rigidbody {
class Joints;
class NodeBone;

class BIORBD_API Markers
{
public:
    Markers();
    virtual ~Markers();

    // Set and get
    void addMarker(
            const Eigen::Vector3d &pos = Eigen::Vector3d(0,0,0),
            const biorbd::utils::String &name = "",
            const biorbd::utils::String &parentName = "",
            bool technical = true,
            bool anatomical = false,
            const biorbd::utils::String& axesToRemove = biorbd::utils::String(),
            int id = -1); // Ajouter un nouveau marker
    const biorbd::rigidbody::NodeBone& marker(
            const unsigned int &i) const;
    std::vector<biorbd::rigidbody::NodeBone> marker(
            const biorbd::rigidbody::Joints& model,
            const unsigned int &idxBone) const;
    std::vector<biorbd::utils::String> markerNames() const;
    std::vector<biorbd::utils::String> technicalMarkerNames() const;
    std::vector<biorbd::utils::String> anatomicalMarkerNames() const;

    static biorbd::rigidbody::NodeBone Tags(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord& Q,
            const biorbd::rigidbody::NodeBone& node,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    biorbd::rigidbody::NodeBone Tags(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord& Q,
            const unsigned int& idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    biorbd::rigidbody::NodeBone Tags(
            const unsigned int& idx,
            bool removeAxis=true); // Retour d'un marqueur ind idx
    std::vector<biorbd::rigidbody::NodeBone> Tags(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> Tags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    biorbd::rigidbody::NodeBone TagsVelocity(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord&,
            const biorbd::utils::GenCoord &Qdot,
            const unsigned int& idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    std::vector<biorbd::rigidbody::NodeBone> TagsVelocity(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            const biorbd::utils::GenCoord &Qdot,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> technicalTags(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> technicalTags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> anatomicalTags(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> anatomicalTags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<biorbd::rigidbody::NodeBone> TechnicalTagsInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs techniques dans leur body
    std::vector<biorbd::rigidbody::NodeBone> AnatomicalTagsInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs anatomiques dans leur body
    std::vector<biorbd::rigidbody::NodeBone> segmentTags(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord&,
            const unsigned int& idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs d'un segment


    unsigned int nTags() const; // Retourne le nombre de marqueurs
    unsigned int nTags(
            biorbd::rigidbody::Joints& model,
            unsigned int idxSegment = static_cast<unsigned int>(-1)) const; // Retourne le nombre de marqueurs du segment idxSegment. Si aucun, somme de tous
    unsigned int nTechTags(); // Retourne le nombre de marqueurs techniques
    unsigned int nTechTags(
            biorbd::rigidbody::Joints& model,
            unsigned int idxSegment); // Retourne le nombre de marqueurs techniques pour le segment idxSegment
    unsigned int nAnatTags(); // Retourne le nombre de marqueurs anatomiques
    std::vector<biorbd::utils::Matrix> TagsJacobian(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Tags
    std::vector<biorbd::utils::Matrix> TechnicalTagsJacobian(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Tags pour les marqueurs techniques
    static biorbd::utils::Matrix TagsJacobian(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            const biorbd::utils::String& parentName,
            const Eigen::Vector3d& p,
            bool updateKin); // Jacobienne d'un marqueur au choix

protected:
    std::vector<biorbd::utils::Matrix> TagsJacobian(
            biorbd::rigidbody::Joints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis,
            bool updateKin,
            bool lookForTechnical); // Retourne la jacobienne des Tags

    std::vector <biorbd::rigidbody::NodeBone> m_marks;

};

}}

#endif // BIORBD_RIGIDBODY_MARKERS_H
