#define BIORBD_API_EXPORTS
#include "../include/s2mMarkers.h"

s2mMarkers::s2mMarkers()
{
    //ctor
}

s2mMarkers::~s2mMarkers()
{
    //dtor
}

// Ajouter un nouveau marker au pool de markers
void s2mMarkers::addMarker(const Eigen::Vector3d &pos,
                           const s2mString &name,
                           const s2mString &parentName,
                           bool technical,
                           bool anatomical,
                           const s2mString& axesToRemove,
                           const int &id)
{
    s2mNodeBone tp(pos, name, parentName, technical, anatomical, axesToRemove, id);
    m_marks.push_back(tp);
}

s2mNodeBone s2mMarkers::marker(const unsigned int &i) const
{
    return *(m_marks.begin()+i);
}

std::vector<s2mNodeBone> s2mMarkers::marker(const s2mJoints& model, const unsigned int &idxBone) const
{
    // Nom du segment a trouver
    s2mString name(model.bone(idxBone).name());

    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!marker(i).parent().compare(name))
            pos.push_back(marker(i));

    return pos;
}

// Se faire renvoyer un marqueur
s2mNodeBone s2mMarkers::Tags(s2mJoints &model, const s2mGenCoord &Q, const s2mNodeBone &n, bool removeAxis, bool updateKin)
{
    unsigned int id = model.GetBodyId(n.parent().c_str());
    s2mNodeBone pos(n);
    pos.setPosition(n.position(removeAxis));

    pos.setPosition(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, pos, updateKin));
    return pos;
}

// Se faire renvoyer un marker
s2mNodeBone s2mMarkers::Tags(s2mJoints& model, const s2mGenCoord &Q, const unsigned int &idx, bool removeAxis, bool updateKin){
    s2mNodeBone node = marker(idx);
    unsigned int id = model.GetBodyId(node.parent().c_str());

    // Récupérer la position du marker dans le repère local
    s2mNodeBone pos = Tags(idx, removeAxis);

    pos.setPosition(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, pos, updateKin));
    return pos;
}
// Se faire renvoyer un marker
s2mNodeBone s2mMarkers::Tags(const unsigned int &idx, bool removeAxis){
    s2mNodeBone node = marker(idx);
    s2mNodeBone pos = node.position(removeAxis);

    // Retourner la position
    return pos;
}

// Se faire renvoyer les markers
std::vector<s2mNodeBone> s2mMarkers::Tags(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis, bool updateKin){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if (i==0)
            pos.push_back(Tags(model, Q, i, removeAxis, updateKin));// Forward kinematics
        else
            pos.push_back(Tags(model, Q, i, removeAxis, false));// Forward kinematics

    return pos;
}
// Se faire renvoyer les markers dans le repère local
std::vector<s2mNodeBone> s2mMarkers::Tags(bool removeAxis){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        pos.push_back(Tags(i, removeAxis));// Forward kinematics

    return pos;
}

// Se faire renvoyer un marker
Eigen::Vector3d s2mMarkers::TagsVelocity(s2mJoints& model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, const unsigned int &idx, bool removeAxis, bool updateKin){
    s2mNodeBone node = marker(idx);
    unsigned int id = model.GetBodyId(node.parent().c_str());

    // Récupérer la position du marker dans le repère local
    s2mNodeBone pos(Tags(idx, removeAxis));

    // Calcul de la vitesse du point
    return RigidBodyDynamics::CalcPointVelocity(model, Q, Qdot, id, pos, updateKin);
}

// Se faire renvoyer les markers
std::vector<Eigen::Vector3d> s2mMarkers::TagsVelocity(s2mJoints& model, const s2mGenCoord &Q, const s2mGenCoord &Qdot, bool removeAxis, bool updateKin){
    std::vector<Eigen::Vector3d> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if (i==0)
            pos.push_back(TagsVelocity(model, Q, Qdot, i, removeAxis, updateKin));// Forward kinematics
        else
            pos.push_back(TagsVelocity(model, Q, Qdot, i, removeAxis, false));// Forward kinematics

    return pos;
}

// Se faire renvoyer les markers techniques
std::vector<s2mNodeBone> s2mMarkers::technicalTags(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis, bool updateKin){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isTechnical() ){
            pos.push_back(Tags(model, Q, i, removeAxis, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les markers techniques dans le repère local
std::vector<s2mNodeBone> s2mMarkers::technicalTags(bool removeAxis){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isTechnical() )
            pos.push_back(Tags(i, removeAxis));// Forward kinematics
    return pos;
}
// Se faire renvoyer les markers anatomiques
std::vector<s2mNodeBone> s2mMarkers::anatomicalTags(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis, bool updateKin){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isAnatomical() ){
            pos.push_back(Tags(model, Q, i, removeAxis, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les markers anatomiques  dans le repère local
std::vector<s2mNodeBone> s2mMarkers::anatomicalTags(bool removeAxis){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isAnatomical() )
            pos.push_back(Tags(i, removeAxis));// Forward kinematics
    return pos;
}
// Se faire renvoyer les markers techniques (duplicat obsolète de technicalTags())
std::vector<s2mNodeBone> s2mMarkers::TechnicalTagsInLocal(bool removeAxis){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isTechnical() )
            pos.push_back( Tags(i, removeAxis) );
    return pos;
}
// Se faire renvoyer les markers anatomiques
std::vector<s2mNodeBone> s2mMarkers::AnatomicalTagsInLocal(bool removeAxis){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isAnatomical() )
            pos.push_back( Tags(i, removeAxis) );
    return pos;
}


std::vector<s2mNodeBone> s2mMarkers::segmentTags(s2mJoints& model, const s2mGenCoord &Q, const unsigned int &idx, bool removeAxis, bool updateKin){
    // Update de la cinématique
    if (updateKin)
        RigidBodyDynamics::UpdateKinematicsCustom(model, &Q,nullptr, nullptr);

    // Nom du segment a trouver
    s2mString name(model.bone(idx).name());

    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!(m_marks.begin()+i)->parent().compare(name))
            pos.push_back(Tags(model,Q,i,removeAxis,false));

    return pos;
}


unsigned int s2mMarkers::nTags() const {
    return m_marks.size();
}

unsigned int s2mMarkers::nTags(s2mJoints& model, unsigned int idxSegment) const {
    // Nom du segment a trouver
    s2mString name(model.bone(idxSegment).name());

    unsigned int n = 0;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!(m_marks.begin()+i)->parent().compare(name))
            ++n;

    return n;
}

// Se faire renvoyer la jacobienne des markers
std::vector<s2mMatrix> s2mMarkers::TagsJacobian(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis, bool updateKin){
    return TagsJacobian(model, Q, removeAxis, updateKin, false);
}

std::vector<s2mMatrix> s2mMarkers::TechnicalTagsJacobian(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis, bool updateKin){
    return TagsJacobian(model, Q, removeAxis, updateKin, true);
}

// Se faire renvoyer la jacobienne des marker techniques
s2mMatrix s2mMarkers::TagsJacobian(s2mJoints& model, const s2mGenCoord &Q, const s2mString& parentName, const Eigen::Vector3d& p, bool updateKin){
    s2mMatrix G(s2mMatrix::Zero(3,model.nbQ()));;

    // Calcul de la jacobienne de ce Tag
    unsigned int id = model.GetBodyId(parentName.c_str());
    RigidBodyDynamics::CalcPointJacobian(model, Q, id, p, G, updateKin);

    return G;
}

// Se faire renvoyer la jacobienne des marker techniques
std::vector<s2mMatrix> s2mMarkers::TagsJacobian(s2mJoints& model, const s2mGenCoord &Q, bool removeAxis, bool updateKin, bool lookForTechnical){
    std::vector<s2mMatrix> G;

    unsigned int idx2(0);
    for (unsigned int idx=0; idx<nTags(); ++idx){
        // Marqueur actuel
        s2mNodeBone node = marker(idx);
        if (lookForTechnical && !node.isTechnical())
            continue;

        unsigned int id = model.GetBodyId(node.parent().c_str());
        Eigen::Vector3d pos = Tags(idx, removeAxis);
        s2mMatrix G_tp(s2mMatrix::Zero(3,model.nbQ()));

        // Calcul de la jacobienne de ce Tag
        if (idx2==0)
            RigidBodyDynamics::CalcPointJacobian(model, Q, id, pos, G_tp, updateKin);
        else
            RigidBodyDynamics::CalcPointJacobian(model, Q, id, pos, G_tp, false); // False for speed

        G.push_back(G_tp);
        ++idx2;
    }

    return G;
}

unsigned int s2mMarkers::nTechTags(){
    unsigned int nTechTags = 0;
    if (nTechTags == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <s2mNodeBone>::iterator it = m_marks.begin(); it!=m_marks.end(); ++it)
            if ((*it).isTechnical())
                ++nTechTags;

    return nTechTags;
}

unsigned int s2mMarkers::nTechTags(s2mJoints& model, unsigned int idxSegment){
    unsigned int nTechTags = 0;

    // Nom du segment a trouver
    s2mString name(model.bone(idxSegment).name());

    if (nTechTags == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <s2mNodeBone>::iterator it = m_marks.begin(); it!=m_marks.end(); ++it)
            if ((*it).isTechnical() && !(*it).parent().compare(name))
                ++nTechTags;

    return nTechTags;
}


unsigned int s2mMarkers::nAnatTags(){
    unsigned int nAnatTags = 0;
    if (nAnatTags == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <s2mNodeBone>::iterator it = m_marks.begin(); it!=m_marks.end(); ++it)
            if ((*it).isAnatomical())
                ++nAnatTags;

    return nAnatTags;
}

std::vector<s2mString> s2mMarkers::markerNames(){
    // Extrait le nom de tous les markers d'un modele
    std::vector<s2mString> names;
    for (unsigned int i=0; i<nTags(); ++i)
        names.push_back(marker(i).name());

    return names;
}

std::vector<s2mString> s2mMarkers::technicalMarkerNames(){
    // Extrait le nom de tous les markers d'un modele
    std::vector<s2mString> names;
    for (unsigned int i=0; i<nTags(); ++i)
        if (marker(i).isTechnical())
            names.push_back(marker(i).name());

    return names;
}

std::vector<s2mString> s2mMarkers::anatomicalMarkerNames(){
    // Extrait le nom de tous les markers d'un modele
    std::vector<s2mString> names;
    for (unsigned int i=0; i<nTags(); ++i)
        if (marker(i).isAnatomical())
            names.push_back(marker(i).name());

    return names;
}
