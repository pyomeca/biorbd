#define BIORBD_API_EXPORTS
#include "RigidBody/IMUs.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "Utils/Rotation.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Bone.h"
#include "RigidBody/IMU.h"

biorbd::rigidbody::IMUs::IMUs() :
    m_IMUs(std::make_shared<std::vector<biorbd::rigidbody::IMU>>())
{
    //ctor
}

biorbd::rigidbody::IMUs::IMUs(const biorbd::rigidbody::IMUs &other)
{
    m_IMUs = other.m_IMUs;
}

biorbd::rigidbody::IMUs::~IMUs()
{

}

biorbd::rigidbody::IMUs biorbd::rigidbody::IMUs::DeepCopy() const
{
    biorbd::rigidbody::IMUs copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::IMUs::DeepCopy(const biorbd::rigidbody::IMUs &other)
{
    m_IMUs->resize(other.m_IMUs->size());
    for (unsigned int i=0; i<other.m_IMUs->size(); ++i)
        (*m_IMUs)[i] = (*other.m_IMUs)[i].DeepCopy();
}

void biorbd::rigidbody::IMUs::addIMU(
        bool technical,
        bool anatomical)
{
    m_IMUs->push_back(biorbd::rigidbody::IMU(technical, anatomical));
}

// Ajouter un nouveau marker au pool de markers
void biorbd::rigidbody::IMUs::addIMU(
        const biorbd::utils::RotoTransNode &RotoTrans,
        bool technical,
        bool anatomical)
{
    m_IMUs->push_back(biorbd::rigidbody::IMU(RotoTrans, technical, anatomical));
}

unsigned int biorbd::rigidbody::IMUs::nIMUs() const
{
    return static_cast<unsigned int>(m_IMUs->size());
}


// Se faire renvoyer les markers dans le repère local
const std::vector<biorbd::rigidbody::IMU>& biorbd::rigidbody::IMUs::IMU() const
{
    return *m_IMUs;
}

std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::IMU(const biorbd::utils::String& segmentName)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i) // passer tous les markers et sélectionner les bons
        if (!IMU(i).parent().compare(segmentName))
            pos.push_back(IMU(i));
    return pos;
}

const biorbd::rigidbody::IMU& biorbd::rigidbody::IMUs::IMU(unsigned int i)
{
    return (*m_IMUs)[i];
}

// Se faire renvoyer les IMUs à la position donnée par Q
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::IMU(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if (i==0)
            pos.push_back(IMU(Q, i, updateKin));// Forward kinematics
        else
            pos.push_back(IMU(Q, i, false));// Forward kinematics

    return pos;
}

// Se faire renvoyer un IMU à la position donnée par Q
biorbd::rigidbody::IMU biorbd::rigidbody::IMUs::IMU(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx,
        bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    if (updateKin)
        model.UpdateKinematicsCustom (&Q, nullptr, nullptr);

    biorbd::rigidbody::IMU node = IMU(idx);
    unsigned int id = static_cast<unsigned int>(model.GetBodyBiorbdId(node.parent()));

    return model.globalJCS(id) * node;
}

// Se faire renvoyer les IMUs techniques
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::technicalIMU(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isTechnical() ){
            pos.push_back(IMU(Q, i, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les IMUs techniques dans le repère local
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::technicalIMU()
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isTechnical() )
            pos.push_back(IMU(i));// Forward kinematics
    return pos;
}

// Se faire renvoyer les IMUs anatomiques
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::anatomicalIMU(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isAnatomical() ){
            pos.push_back(IMU(Q, i, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les IMUs anatomiques dans le repère local
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::anatomicalIMU()
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isAnatomical() )
            pos.push_back(IMU(i));// Forward kinematics
    return pos;
}

std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::segmentIMU(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx,
        bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    // Update de la cinématique
    if (updateKin)
        model.UpdateKinematicsCustom(&Q,nullptr, nullptr);

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idx).name());

    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i) // passer tous les markers et sélectionner les bons
        if (!((*m_IMUs)[i]).parent().compare(name))
            pos.push_back(IMU(Q,i,false));

    return pos;
}

// Se faire renvoyer la jacobienne des markers
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::IMUs::IMUJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    return IMUJacobian(Q, updateKin, false);
}

// Se faire renvoyer la jacobienne des marker techniques
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::IMUs::TechnicalIMUJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin)
{
    return IMUJacobian(Q, updateKin, true);
}


// Protected function
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::IMUs::IMUJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin,
        bool lookForTechnical)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    std::vector<biorbd::utils::Matrix> G;

    bool first(true);
    for (unsigned int idx=0; idx<nIMUs(); ++idx){
        // Marqueur actuel
        biorbd::rigidbody::IMU node = IMU(idx);
        if (lookForTechnical && !node.isTechnical())
            continue;

        unsigned int id = model.GetBodyId(node.parent().c_str());
        biorbd::utils::Matrix G_tp(biorbd::utils::Matrix::Zero(9,model.dof_count));

        // Calcul de la jacobienne de ce Tag
        if (first)
            model.CalcMatRotJacobian(Q, id, node.rot(), G_tp, updateKin);
        else
            model.CalcMatRotJacobian(Q, id, node.rot(), G_tp, false); // False for speed

        G.push_back(G_tp);
        first = false;
    }

    return G;
}

unsigned int biorbd::rigidbody::IMUs::nTechIMUs()
{
    unsigned int nTech = 0;
    if (nTech == 0) // Si la fonction n'a jamais été appelée encore
        for (biorbd::rigidbody::IMU imu : *m_IMUs)
            if (imu.isTechnical())
                ++nTech;

    return nTech;
}

unsigned int biorbd::rigidbody::IMUs::nAnatIMUs()
{
    unsigned int nAnat = 0;
    if (nAnat == 0) // Si la fonction n'a jamais été appelée encore
        for (biorbd::rigidbody::IMU imu : *m_IMUs)
            if (imu.isAnatomical())
                ++nAnat;

    return nAnat;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::IMUs::IMUsNames()
{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nIMUs(); ++i)
        names.push_back(IMU(i).name());
    return names;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::IMUs::technicalIMUsNames()
{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if (IMU(i).isTechnical())
            names.push_back(IMU(i).name());

    return names;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::IMUs::anatomicalIMUsNames()
{
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if (IMU(i).isAnatomical())
            names.push_back(IMU(i).name());

    return names;
}
