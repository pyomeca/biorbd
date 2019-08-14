#define BIORBD_API_EXPORTS
#include "RigidBody/Markers.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "Utils/GenCoord.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Patch.h"
#include "RigidBody/NodeBone.h"
#include "RigidBody/Bone.h"


s2mMarkers::s2mMarkers()
{
    //ctor
}

s2mMarkers::~s2mMarkers()
{
    //dtor
}

// Ajouter un nouveau marker au pool de markers
void s2mMarkers::addMarker(
        const Eigen::Vector3d &pos,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        bool technical,
        bool anatomical,
        const biorbd::utils::String& axesToRemove,
        int id)
{
    s2mNodeBone tp(pos, name, parentName, technical, anatomical, axesToRemove, id);
    m_marks.push_back(tp);
}

const s2mNodeBone &s2mMarkers::marker(const unsigned int &i) const
{
    return m_marks[i];
}

std::vector<s2mNodeBone> s2mMarkers::marker(
        const s2mJoints& model,
        const unsigned int &idxBone) const
{
    // Nom du segment a trouver
    const biorbd::utils::String& name(model.bone(idxBone).name());

    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!marker(i).parent().compare(name))
            pos.push_back(marker(i));

    return pos;
}

// Se faire renvoyer un marqueur
s2mNodeBone s2mMarkers::Tags(
        s2mJoints &model,
        const biorbd::utils::GenCoord &Q,
        const s2mNodeBone &n,
        bool removeAxis,
        bool updateKin)
{
    unsigned int id = model.GetBodyId(n.parent().c_str());
    s2mNodeBone pos(n);
    pos.setPosition(n.position(removeAxis));

    pos.setPosition(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, pos, updateKin));
    return pos;
}

// Se faire renvoyer un marker
s2mNodeBone s2mMarkers::Tags(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const unsigned int &idx,
        bool removeAxis,
        bool updateKin){
    s2mNodeBone node = marker(idx);
    unsigned int id = model.GetBodyId(node.parent().c_str());

    // Récupérer la position du marker dans le repère local
    s2mNodeBone pos = Tags(idx, removeAxis);

    pos.setPosition(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, pos, updateKin));
    return pos;
}
// Se faire renvoyer un marker
s2mNodeBone s2mMarkers::Tags(
        const unsigned int &idx,
        bool removeAxis){
    s2mNodeBone node = marker(idx);
    s2mNodeBone pos = node.position(removeAxis);

    // Retourner la position
    return pos;
}

// Se faire renvoyer les markers
std::vector<s2mNodeBone> s2mMarkers::Tags(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool removeAxis,
        bool updateKin){
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
s2mNodeBone s2mMarkers::TagsVelocity(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const biorbd::utils::GenCoord &Qdot,
        const unsigned int &idx,
        bool removeAxis,
        bool updateKin){
    s2mNodeBone node = marker(idx);
    unsigned int id = model.GetBodyId(node.parent().c_str());

    // Récupérer la position du marker dans le repère local
    s2mNodeBone pos(Tags(idx, removeAxis));

    // Calcul de la vitesse du point
    pos.setPosition(RigidBodyDynamics::CalcPointVelocity(model, Q, Qdot, id, pos, updateKin));
    return pos;
}

// Se faire renvoyer les markers
std::vector<s2mNodeBone> s2mMarkers::TagsVelocity(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const biorbd::utils::GenCoord &Qdot,
        bool removeAxis,
        bool updateKin){
    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if (i==0)
            pos.push_back(TagsVelocity(model, Q, Qdot, i, removeAxis, updateKin));// Forward kinematics
        else
            pos.push_back(TagsVelocity(model, Q, Qdot, i, removeAxis, false));// Forward kinematics

    return pos;
}

// Se faire renvoyer les markers techniques
std::vector<s2mNodeBone> s2mMarkers::technicalTags(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool removeAxis,
        bool updateKin){
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
std::vector<s2mNodeBone> s2mMarkers::anatomicalTags(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool removeAxis,
        bool updateKin){
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


std::vector<s2mNodeBone> s2mMarkers::segmentTags(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const unsigned int &idx,
        bool removeAxis,
        bool updateKin){
    // Update de la cinématique
    if (updateKin)
        RigidBodyDynamics::UpdateKinematicsCustom(model, &Q,nullptr, nullptr);

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idx).name());

    std::vector<s2mNodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!(m_marks.begin()+i)->parent().compare(name))
            pos.push_back(Tags(model,Q,i,removeAxis,false));

    return pos;
}


unsigned int s2mMarkers::nTags() const {
    return static_cast<unsigned int>(m_marks.size());
}

unsigned int s2mMarkers::nTags(s2mJoints& model, unsigned int idxSegment) const {
    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idxSegment).name());

    unsigned int n = 0;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!(m_marks.begin()+i)->parent().compare(name))
            ++n;

    return n;
}

// Se faire renvoyer la jacobienne des markers
std::vector<biorbd::utils::Matrix> s2mMarkers::TagsJacobian(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool removeAxis,
        bool updateKin){
    return TagsJacobian(model, Q, removeAxis, updateKin, false);
}

std::vector<biorbd::utils::Matrix> s2mMarkers::TechnicalTagsJacobian(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool removeAxis,
        bool updateKin){
    return TagsJacobian(model, Q, removeAxis, updateKin, true);
}

// Se faire renvoyer la jacobienne des marker techniques
biorbd::utils::Matrix s2mMarkers::TagsJacobian(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const biorbd::utils::String& parentName,
        const Eigen::Vector3d& p,
        bool updateKin){
    biorbd::utils::Matrix G(biorbd::utils::Matrix::Zero(3,model.nbQ()));;

    // Calcul de la jacobienne de ce Tag
    unsigned int id = model.GetBodyId(parentName.c_str());
    RigidBodyDynamics::CalcPointJacobian(model, Q, id, p, G, updateKin);

    return G;
}

// Se faire renvoyer la jacobienne des marker techniques
std::vector<biorbd::utils::Matrix> s2mMarkers::TagsJacobian(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool removeAxis,
        bool updateKin,
        bool lookForTechnical){
    std::vector<biorbd::utils::Matrix> G;

    unsigned int idx2(0);
    for (unsigned int idx=0; idx<nTags(); ++idx){
        // Marqueur actuel
        s2mNodeBone node = marker(idx);
        if (lookForTechnical && !node.isTechnical())
            continue;

        unsigned int id = model.GetBodyId(node.parent().c_str());
        Eigen::Vector3d pos = Tags(idx, removeAxis);
        biorbd::utils::Matrix G_tp(biorbd::utils::Matrix::Zero(3,model.nbQ()));

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
    biorbd::utils::String name(model.bone(idxSegment).name());

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

std::vector<biorbd::utils::String> s2mMarkers::markerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nTags(); ++i)
        names.push_back(marker(i).name());

    return names;
}

std::vector<biorbd::utils::String> s2mMarkers::technicalMarkerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nTags(); ++i)
        if (marker(i).isTechnical())
            names.push_back(marker(i).name());

    return names;
}

std::vector<biorbd::utils::String> s2mMarkers::anatomicalMarkerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nTags(); ++i)
        if (marker(i).isAnatomical())
            names.push_back(marker(i).name());

    return names;
}
