#ifndef S2MMARKERS_H
#define S2MMARKERS_H
    #include "s2mMatrix.h"
    #include "biorbdConfig.h"
    #include "s2mString.h"
    #include <rbdl/rbdl.h>
    #include "s2mBone.h"
    #include "s2mNodeBone.h"
    #include "s2mGenCoord.h"

class s2mGenCoord;
class s2mJoints;
class BIORBD_API s2mMarkers
{
    public:
        s2mMarkers();
        ~s2mMarkers();

        // Set and get
        void addMarker(const Eigen::Vector3d &pos = Eigen::Vector3d(0,0,0),
                       const s2mString &name = "",
                       const s2mString &parentName = "",
                       bool technical = true,
                       bool anatomical = false,
                       const s2mString& axesToRemove = s2mString(),
                       const int &id = -1); // Ajouter un nouveau marker
        s2mNodeBone marker(const unsigned int &i) const;
        std::vector<s2mNodeBone> marker(const s2mJoints& model, const unsigned int &idxBone) const;
        std::vector<s2mString> markerNames();
        std::vector<s2mString> technicalMarkerNames();
        std::vector<s2mString> anatomicalMarkerNames();

        static s2mNodeBone Tags(s2mJoints& model, const s2mGenCoord&, const s2mNodeBone&, bool removeAxis=true, bool updateKin = true); // Retour d'un marqueur ind idx
        s2mNodeBone Tags(s2mJoints& model, const s2mGenCoord&, const unsigned int&, bool removeAxis=true, bool updateKin = true); // Retour d'un marqueur ind idx
        s2mNodeBone Tags(const unsigned int&, bool removeAxis=true); // Retour d'un marqueur ind idx
        std::vector<s2mNodeBone> Tags(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis=true, bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
        std::vector<s2mNodeBone> Tags(bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
        Eigen::Vector3d  TagsVelocity(s2mJoints& model, const s2mGenCoord&, const s2mGenCoord &Qdot, const unsigned int&, bool removeAxis=true, bool updateKin = true); // Retour d'un marqueur ind idx
        std::vector<Eigen::Vector3d > TagsVelocity(s2mJoints& model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, bool removeAxis=true, bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
        std::vector<s2mNodeBone> technicalTags(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis=true, bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
        std::vector<s2mNodeBone> technicalTags(bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
        std::vector<s2mNodeBone> anatomicalTags(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis=true, bool updateKin = true); // Retour d'un STL vector de tous les marqueurs
        std::vector<s2mNodeBone> anatomicalTags(bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs
        std::vector<s2mNodeBone> TechnicalTagsInLocal(bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs techniques dans leur body
        std::vector<s2mNodeBone> AnatomicalTagsInLocal(bool removeAxis=true); // Retour d'un STL vector de tous les marqueurs anatomiques dans leur body
        std::vector<s2mNodeBone> segmentTags(s2mJoints& model, const s2mGenCoord&, const unsigned int&, bool removeAxis=true, bool updateKin = true); // Retour d'un STL vector de tous les marqueurs d'un segment


        unsigned int nTags() const; // Retourne le nombre de marqueurs
        unsigned int nTags(s2mJoints& model, unsigned int idxSegment = -1) const; // Retourne le nombre de marqueurs du segment idxSegment. Si aucun, somme de tous
        unsigned int nTechTags(); // Retourne le nombre de marqueurs techniques
        unsigned int nTechTags(s2mJoints& model, unsigned int idxSegment); // Retourne le nombre de marqueurs techniques pour le segment idxSegment
        unsigned int nAnatTags(); // Retourne le nombre de marqueurs anatomiques
        std::vector<s2mMatrix> TagsJacobian(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis=true, bool updateKin = true); // Retourne la jacobienne des Tags
        std::vector<s2mMatrix> TechnicalTagsJacobian(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis=true, bool updateKin = true); // Retourne la jacobienne des Tags pour les marqueurs techniques
        static s2mMatrix TagsJacobian(s2mJoints& model, const s2mGenCoord &Q, const s2mString& parentName, const Eigen::Vector3d& p, bool updateKin); // Jacobienne d'un marqueur au choix

protected:
        std::vector<s2mMatrix> TagsJacobian(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis, bool updateKin, bool lookForTechnical); // Retourne la jacobienne des Tags

        std::vector <s2mNodeBone> m_marks;
    private:
};

#endif // S2MMARKERS_H
