#define BIORBD_API_EXPORTS
#include "RigidBody/Markers.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"
#include "RigidBody/NodeBone.h"
#include "RigidBody/Bone.h"


biorbd::rigidbody::Markers::Markers() :
    m_marks(std::make_shared<std::vector<biorbd::rigidbody::NodeBone>>())
{
    //ctor
}

biorbd::rigidbody::Markers::Markers(const biorbd::rigidbody::Markers &other) :
    m_marks(other.m_marks)
{

}

biorbd::rigidbody::Markers::~Markers()
{
    //dtor
}

biorbd::rigidbody::Markers biorbd::rigidbody::Markers::DeepCopy() const
{
    biorbd::rigidbody::Markers copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Markers::DeepCopy(const biorbd::rigidbody::Markers &other)
{
    m_marks->resize(other.m_marks->size());
    for (unsigned int i=0; i<other.m_marks->size(); ++i)
        (*m_marks)[i] = (*other.m_marks)[i].DeepCopy();
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
    m_marks->push_back(tp);
}

const biorbd::rigidbody::NodeBone &biorbd::rigidbody::Markers::marker(
        unsigned int i) const
{
    return (*m_marks)[i];
}

std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::marker(
        const biorbd::utils::String& segmentName) const
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i) // passer tous les markers et sélectionner les bons
        if (!marker(i).parent().compare(segmentName))
            pos.push_back(marker(i));

    return pos;
}

// Se faire renvoyer un marqueur
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::marker(
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
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::marker(
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
    biorbd::rigidbody::NodeBone pos = marker(idx, removeAxis);

    return biorbd::rigidbody::NodeBone(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, pos, updateKin));
}
// Se faire renvoyer un marker
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::marker(
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
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::markers(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if (i==0)
            pos.push_back(marker(Q, i, removeAxis, updateKin));// Forward kinematics
        else
            pos.push_back(marker(Q, i, removeAxis, false));// Forward kinematics

    return pos;
}
// Se faire renvoyer les markers dans le repère local
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::markers(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        pos.push_back(marker(i, removeAxis));// Forward kinematics

    return pos;
}

// Se faire renvoyer un marker
biorbd::rigidbody::NodeBone biorbd::rigidbody::Markers::markerVelocity(
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
    biorbd::rigidbody::NodeBone pos(marker(idx, removeAxis));

    // Calcul de la vitesse du point
    return biorbd::rigidbody::NodeBone(RigidBodyDynamics::CalcPointVelocity(model, Q, Qdot, id, pos, updateKin));
}

// Se faire renvoyer les markers
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::markerVelocity(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if (i==0)
            pos.push_back(markerVelocity(Q, Qdot, i, removeAxis, updateKin));// Forward kinematics
        else
            pos.push_back(markerVelocity(Q, Qdot, i, removeAxis, false));// Forward kinematics

    return pos;
}

// Se faire renvoyer les markers techniques
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::technicalMarkers(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if ( marker(i).isTechnical() )
        {
            pos.push_back(marker(Q, i, removeAxis, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les markers techniques dans le repère local
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::technicalMarkers(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if ( marker(i).isTechnical() )
            pos.push_back(marker(i, removeAxis));// Forward kinematics
    return pos;
}
// Se faire renvoyer les markers anatomiques
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::anatomicalMarkers(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if ( marker(i).isAnatomical() ){
            pos.push_back(marker(Q, i, removeAxis, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les markers anatomiques  dans le repère local
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::anatomicalMarkers(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if ( marker(i).isAnatomical() )
            pos.push_back(marker(i, removeAxis));// Forward kinematics
    return pos;
}
// Se faire renvoyer les markers techniques (duplicat obsolète de technicalMarkers())
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::TechnicalMarkersInLocal(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if ( marker(i).isTechnical() )
            pos.push_back( marker(i, removeAxis) );
    return pos;
}
// Se faire renvoyer les markers anatomiques
std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::AnatomicalMarkersInLocal(bool removeAxis)
{
    std::vector<biorbd::rigidbody::NodeBone> pos;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if ( marker(i).isAnatomical() )
            pos.push_back( marker(i, removeAxis) );
    return pos;
}


std::vector<biorbd::rigidbody::NodeBone> biorbd::rigidbody::Markers::segmentMarkers(
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
    for (unsigned int i=0; i<nMarkers(); ++i) // passer tous les markers et sélectionner les bons
        if ((*m_marks)[i].parent().compare(name))
            pos.push_back(marker(Q,i,removeAxis,false));

    return pos;
}


unsigned int biorbd::rigidbody::Markers::nMarkers() const
{
    return static_cast<unsigned int>(m_marks->size());
}

unsigned int biorbd::rigidbody::Markers::nMarkers(unsigned int idxSegment) const
{
    // Assuming that this is also a joint type (via BiorbdModel)
    const biorbd::rigidbody::Joints &model = dynamic_cast<const biorbd::rigidbody::Joints &>(*this);

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idxSegment).name());

    unsigned int n = 0;
    for (unsigned int i=0; i<nMarkers(); ++i) // passer tous les markers et sélectionner les bons
        if ((*m_marks)[i].parent().compare(name))
            ++n;

    return n;
}

// Se faire renvoyer la jacobienne des markers
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Markers::markersJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    return markersJacobian(Q, removeAxis, updateKin, false);
}

std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Markers::TechnicalMarkersJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin)
{
    return markersJacobian(Q, removeAxis, updateKin, true);
}

// Se faire renvoyer la jacobienne des marker techniques
biorbd::utils::Matrix biorbd::rigidbody::Markers::markersJacobian(
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
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::Markers::markersJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin,
        bool lookForTechnical)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    std::vector<biorbd::utils::Matrix> G;

    unsigned int idx2(0);
    for (unsigned int idx=0; idx<nMarkers(); ++idx){
        // Marqueur actuel
        biorbd::rigidbody::NodeBone node = marker(idx);
        if (lookForTechnical && !node.isTechnical())
            continue;

        unsigned int id = model.GetBodyId(node.parent().c_str());
        biorbd::utils::Node3d pos = marker(idx, removeAxis);
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

unsigned int biorbd::rigidbody::Markers::nTechnicalMarkers()
{
    unsigned int nTechMarkers = 0;
    if (nTechMarkers == 0) // Si la fonction n'a jamais été appelée encore
        for (auto mark : *m_marks)
            if (mark.isTechnical())
                ++nTechMarkers;

    return nTechMarkers;
}

unsigned int biorbd::rigidbody::Markers::nTechnicalMarkers(unsigned int idxSegment)
{
    // Assuming that this is also a joint type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    unsigned int nTechMarkers = 0;

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idxSegment).name());

    if (nTechMarkers == 0) // Si la fonction n'a jamais été appelée encore
        for (auto mark : *m_marks)
            if (mark.isTechnical() && !mark.parent().compare(name))
                ++nTechMarkers;

    return nTechMarkers;
}


unsigned int biorbd::rigidbody::Markers::nAnatomicalMarkers()
{
    unsigned int nAnatMarkers = 0;
    if (nAnatMarkers == 0) // Si la fonction n'a jamais été appelée encore
        for (auto mark : *m_marks)
            if (mark.isAnatomical())
                ++nAnatMarkers;

    return nAnatMarkers;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::Markers::markerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nMarkers(); ++i)
        names.push_back(marker(i).name());

    return names;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::Markers::technicalMarkerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if (marker(i).isTechnical())
            names.push_back(marker(i).name());

    return names;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::Markers::anatomicalMarkerNames() const{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nMarkers(); ++i)
        if (marker(i).isAnatomical())
            names.push_back(marker(i).name());

    return names;
}
