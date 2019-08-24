#define BIORBD_API_EXPORTS
#include "RigidBody/Markers.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Patch.h"
#include "RigidBody/NodeBone.h"
#include "RigidBody/Bone.h"


biorbd::rigidbody::Markers::Markers()
{
    //ctor
}

biorbd::rigidbody::Markers::~Markers()
{
    //dtor
}

// Ajouter un nouveau marker au pool de markers
void biorbd::rigidbody::Markers::addMarker(
        const biorbd::rigidbody::NodeBone &pos,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        bool technical,
        bool anatomical,
        const biorbd::utils::String& axesToRemove,
        int id)
{
    biorbd::rigidbody::NodeBone tp(pos, name, parentName, technical, anatomical, axesToRemove, id);
    m_marks.push_back(tp);
}

const biorbd::rigidbody::NodeBone &biorbd::rigidbody::Markers::marker(
        unsigned int i) const
{
    return m_marks[i];
}

std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::marker(
        const biorbd::utils::String& segmentName) const
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!marker(i).parent().compare(segmentName))
            pos.push_back(marker(i));

    return pos;
}

// Se faire renvoyer un marqueur
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::Tags(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::NodeBone &n,
        bool removeAxis,
        bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    unsigned int id = model.GetBodyId(n.parent().c_str());
    if (removeAxis)
        return biorbd::rigidbody::NodeBone(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, n.removeAxes(), updateKin));
    else
        return biorbd::rigidbody::NodeBone(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, n, updateKin));
}

// Se faire renvoyer un marker
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::Tags(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx,
        bool removeAxis,
        bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    biorbd::rigidbody::NodeBone node = marker(idx);
    unsigned int id = model.GetBodyId(node.parent().c_str());

    // Récupérer la position du marker dans le repère local
    biorbd::rigidbody::NodeBone pos = Tags(idx, removeAxis);

    return biorbd::rigidbody::NodeBone(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, pos, updateKin));
}
// Se faire renvoyer un marker
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::Tags(
        unsigned int idx,
        bool removeAxis)
{
    biorbd::rigidbody::NodeBone node = marker(idx);
    if (removeAxis)
        return node.removeAxes();
    else
        return node;
}

// Se faire renvoyer les markers
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::Tags(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if (i==0)
            pos.push_back(Tags(Q, i, removeAxis, updateKin));// Forward kinematics
        else
            pos.push_back(Tags(Q, i, removeAxis, false));// Forward kinematics

    return pos;
}
// Se faire renvoyer les markers dans le repère local
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::Tags(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        pos.push_back(Tags(i, removeAxis));// Forward kinematics

    return pos;
}

// Se faire renvoyer un marker
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::TagsVelocity(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        unsigned int idx,
        bool removeAxis,
        bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    biorbd::rigidbody::NodeBone node = marker(idx);
    unsigned int id = model.GetBodyId(node.parent().c_str());

    // Récupérer la position du marker dans le repère local
    biorbd::rigidbody::NodeBone pos(Tags(idx, removeAxis));

    // Calcul de la vitesse du point
    return biorbd::rigidbody::NodeBone(RigidBodyDynamics::CalcPointVelocity(model, Q, Qdot, id, pos, updateKin));
}

// Se faire renvoyer les markers
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::TagsVelocity(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if (i==0)
            pos.push_back(TagsVelocity(Q, Qdot, i, removeAxis, updateKin));// Forward kinematics
        else
            pos.push_back(TagsVelocity(Q, Qdot, i, removeAxis, false));// Forward kinematics

    return pos;
}

// Se faire renvoyer les markers techniques
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::technicalTags(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isTechnical() )
        {
            pos.push_back(Tags(Q, i, removeAxis, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les markers techniques dans le repère local
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::technicalTags(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isTechnical() )
            pos.push_back(Tags(i, removeAxis));// Forward kinematics
    return pos;
}
// Se faire renvoyer les markers anatomiques
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::anatomicalTags(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isAnatomical() ){
            pos.push_back(Tags(Q, i, removeAxis, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les markers anatomiques  dans le repère local
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::anatomicalTags(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isAnatomical() )
            pos.push_back(Tags(i, removeAxis));// Forward kinematics
    return pos;
}
// Se faire renvoyer les markers techniques (duplicat obsolète de technicalTags())
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::TechnicalTagsInLocal(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isTechnical() )
            pos.push_back( Tags(i, removeAxis) );
    return pos;
}
// Se faire renvoyer les markers anatomiques
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::AnatomicalTagsInLocal(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i)
        if ( marker(i).isAnatomical() )
            pos.push_back( Tags(i, removeAxis) );
    return pos;
}


std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::segmentTags(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx,
        bool removeAxis,
        bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    // Update de la cinématique
    if (updateKin)
        model.UpdateKinematicsCustom(&Q, nullptr, nullptr);

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idx).name());

    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!(m_marks.begin()+i)->parent().compare(name))
            pos.push_back(Tags(Q,i,removeAxis,false));

    return pos;
}


unsigned int biorbd::rigidbody::Markers::nTags() const
{
    return static_cast<unsigned int>(m_marks.size());
}

unsigned int biorbd::rigidbody::Markers::nTags(unsigned int idxSegment) const
{
    // Assuming that this is also a joint type (via BiorbdModel)
    const biorbd::rigidbody::Joints &model = dynamic_cast<const biorbd::rigidbody::Joints &>(*this);

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idxSegment).name());

    unsigned int n = 0;
    for (unsigned int i=0; i<nTags(); ++i) // passer tous les markers et sélectionner les bons
        if (!(m_marks.begin()+i)->parent().compare(name))
            ++n;

    return n;
}

// Se faire renvoyer la jacobienne des markers
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Markers::TagsJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    return TagsJacobian(Q, removeAxis, updateKin, false);
}

std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Markers::TechnicalTagsJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    return TagsJacobian(Q, removeAxis, updateKin, true);
}

// Se faire renvoyer la jacobienne des marker techniques
biorbd::utils::Matrix biorbd::rigidbody::Markers::TagsJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::utils::String& parentName,
        const biorbd::rigidbody::NodeBone& p,
        bool updateKin)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    biorbd::utils::Matrix G(biorbd::utils::Matrix::Zero(3, model.nbQ()));;

    // Calcul de la jacobienne de ce Tag
    unsigned int id = model.GetBodyId(parentName.c_str());
    RigidBodyDynamics::CalcPointJacobian(model, Q, id, p, G, updateKin);

    return G;
}

// Se faire renvoyer la jacobienne des marker techniques
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Markers::TagsJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin,
        bool lookForTechnical)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    std::vector<biorbd::utils::Matrix> G;

    unsigned int idx2(0);
    for (unsigned int idx=0; idx<nTags(); ++idx){
        // Marqueur actuel
        biorbd::rigidbody::NodeBone node = marker(idx);
        if (lookForTechnical && !node.isTechnical())
            continue;

        unsigned int id = model.GetBodyId(node.parent().c_str());
        biorbd::utils::Node3d pos = Tags(idx, removeAxis);
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

unsigned int biorbd::rigidbody::Markers::nTechTags()
{
    unsigned int nTechTags = 0;
    if (nTechTags == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <biorbd::rigidbody::NodeBone>::iterator it = m_marks.begin(); it!=m_marks.end(); ++it)
            if ((*it).isTechnical())
                ++nTechTags;

    return nTechTags;
}

unsigned int biorbd::rigidbody::Markers::nTechTags(unsigned int idxSegment)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    unsigned int nTechTags = 0;

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idxSegment).name());

    if (nTechTags == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <biorbd::rigidbody::NodeBone>::iterator it = m_marks.begin(); it!=m_marks.end(); ++it)
            if ((*it).isTechnical() && !(*it).parent().compare(name))
                ++nTechTags;

    return nTechTags;
}


unsigned int biorbd::rigidbody::Markers::nAnatTags()
{
    unsigned int nAnatTags = 0;
    if (nAnatTags == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <biorbd::rigidbody::NodeBone>::iterator it = m_marks.begin(); it!=m_marks.end(); ++it)
            if ((*it).isAnatomical())
                ++nAnatTags;

    return nAnatTags;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::Markers::markerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nTags(); ++i)
        names.push_back(marker(i).name());

    return names;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::Markers::technicalMarkerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nTags(); ++i)
        if (marker(i).isTechnical())
            names.push_back(marker(i).name());

    return names;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::Markers::anatomicalMarkerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nTags(); ++i)
        if (marker(i).isAnatomical())
            names.push_back(marker(i).name());

    return names;
}
