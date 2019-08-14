#ifndef S2M_MARKERS_H
#define S2M_MARKERS_H

#include <vector>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd { namespace utils {
class Matrix;
class GenCoord;
}}
class s2mJoints;
class s2mNodeBone;
class BIORBD_API s2mMarkers
{
public:
    s2mMarkers();
    virtual ~s2mMarkers();

    // Set and get
    void addMarker(
            const Eigen::Vector3d &pos = Eigen::Vector3d(0,0,0),
            const biorbd::utils::String &name = "",
            const biorbd::utils::String &parentName = "",
            bool technical = true,
            bool anatomical = false,
            const biorbd::utils::String& axesToRemove = biorbd::utils::String(),
            int id = -1); // Ajouter un nouveau marker
    const s2mNodeBone& marker(
            const unsigned int &i) const;
    std::vector<s2mNodeBone> marker(
            const s2mJoints& model,
            const unsigned int &idxBone) const;
    std::vector<biorbd::utils::String> markerNames() const;
    std::vector<biorbd::utils::String> technicalMarkerNames() const;
    std::vector<biorbd::utils::String> anatomicalMarkerNames() const;

    static s2mNodeBone Tags(
            s2mJoints& model,
            const biorbd::utils::GenCoord& Q,
            const s2mNodeBone& node,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    s2mNodeBone Tags(
            s2mJoints& model,
            const biorbd::utils::GenCoord& Q,
            const unsigned int& idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    s2mNodeBone Tags(
            const unsigned int& idx,
            bool removeAxis=true); // Retour d'un marqueur ind idx
    std::vector<s2mNodeBone> Tags(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<s2mNodeBone> Tags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    s2mNodeBone TagsVelocity(
            s2mJoints& model,
            const biorbd::utils::GenCoord&,
            const biorbd::utils::GenCoord &Qdot,
            const unsigned int& idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un marqueur ind idx
    std::vector<s2mNodeBone> TagsVelocity(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            const biorbd::utils::GenCoord &Qdot,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<s2mNodeBone> technicalTags(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<s2mNodeBone> technicalTags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<s2mNodeBone> anatomicalTags(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
    std::vector<s2mNodeBone> anatomicalTags(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
    std::vector<s2mNodeBone> TechnicalTagsInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs techniques dans leur body
    std::vector<s2mNodeBone> AnatomicalTagsInLocal(
            bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs anatomiques dans leur body
    std::vector<s2mNodeBone> segmentTags(
            s2mJoints& model,
            const biorbd::utils::GenCoord&,
            const unsigned int& idx,
            bool removeAxis=true,
            bool updateKin = true); // Retour d'un STL vector de tous les marqueurs d'un segment


    unsigned int nTags() const; // Retourne le nombre de marqueurs
    unsigned int nTags(
            s2mJoints& model,
            unsigned int idxSegment = static_cast<unsigned int>(-1)) const; // Retourne le nombre de marqueurs du segment idxSegment. Si aucun, somme de tous
    unsigned int nTechTags(); // Retourne le nombre de marqueurs techniques
    unsigned int nTechTags(
            s2mJoints& model,
            unsigned int idxSegment); // Retourne le nombre de marqueurs techniques pour le segment idxSegment
    unsigned int nAnatTags(); // Retourne le nombre de marqueurs anatomiques
    std::vector<biorbd::utils::Matrix> TagsJacobian(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Tags
    std::vector<biorbd::utils::Matrix> TechnicalTagsJacobian(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis=true,
            bool updateKin = true); // Retourne la jacobienne des Tags pour les marqueurs techniques
    static biorbd::utils::Matrix TagsJacobian(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            const biorbd::utils::String& parentName,
            const Eigen::Vector3d& p,
            bool updateKin); // Jacobienne d'un marqueur au choix

protected:
    std::vector<biorbd::utils::Matrix> TagsJacobian(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool removeAxis,
            bool updateKin,
            bool lookForTechnical); // Retourne la jacobienne des Tags

    std::vector <s2mNodeBone> m_marks;

};

#endif // S2M_MARKERS_H
