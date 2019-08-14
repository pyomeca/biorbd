#define BIORBD_API_EXPORTS
#include "s2mIMUs.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "s2mJoints.h"
#include "s2mIMU.h"
#include "Utils/GenCoord.h"
#include "Utils/Matrix.h"
#include "s2mBone.h"

s2mIMUs::s2mIMUs()
{
    //ctor
}

s2mIMUs::~s2mIMUs()
{
    //dtor
}

// Ajouter un nouveau marker au pool de markers
void s2mIMUs::addIMU(
        const biorbd::utils::Attitude &pos,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        const bool &technical,
        const bool &anatomical,
        const int &id)
{
    s2mIMU tp(pos, name, parentName, technical, anatomical, id);
    m_IMUs.push_back(tp);
}

unsigned int s2mIMUs::nIMUs() const
{
    return static_cast<unsigned int>(m_IMUs.size());
}


// Se faire renvoyer les markers dans le repère local
std::vector<s2mIMU> s2mIMUs::IMU(){
    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        pos.push_back(IMU(i));// Forward kinematics

    return pos;
}

std::vector<s2mIMU> s2mIMUs::IMU(s2mJoints &m, unsigned int idxBone){
    // Nom du segment a trouver
    biorbd::utils::String name(m.bone(idxBone).name());

    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i) // passer tous les markers et sélectionner les bons
        if (!IMU(i).parent().compare(name))
            pos.push_back(IMU(i));

    return pos;
}

const s2mIMU &s2mIMUs::IMU(const unsigned int &i)
{
    std::vector <s2mIMU>::iterator it;
    it = m_IMUs.begin()+i;
    return *it;
}

// Se faire renvoyer les IMUs à la position donnée par Q
std::vector<s2mIMU> s2mIMUs::IMU(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const bool &updateKin){
    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if (i==0)
            pos.push_back(IMU(model, Q, i, updateKin));// Forward kinematics
        else
            pos.push_back(IMU(model, Q, i, false));// Forward kinematics

    return pos;
}

// Se faire renvoyer un IMU à la position donnée par Q
s2mIMU s2mIMUs::IMU(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const unsigned int &idx,
        const bool &updateKin){
    s2mIMU node = IMU(idx);
    unsigned int id = static_cast<unsigned int>(model.GetBodyS2MId(node.parent()));

    biorbd::utils::Attitude parent(model.globalJCS(Q, id, updateKin));

    s2mIMU node_tp = node;
    node_tp.block(0,0,4,4) = parent * node;
    return node_tp;
}

// Se faire renvoyer les IMUs techniques
std::vector<s2mIMU> s2mIMUs::technicalIMU(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool updateKin){
    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isTechnical() ){
            pos.push_back(IMU(model, Q, i, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les IMUs techniques dans le repère local
std::vector<s2mIMU> s2mIMUs::technicalIMU(){
    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isTechnical() )
            pos.push_back(IMU(i));// Forward kinematics
    return pos;
}

// Se faire renvoyer les IMUs anatomiques
std::vector<s2mIMU> s2mIMUs::anatomicalIMU(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        bool updateKin){
    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isAnatomical() ){
            pos.push_back(IMU(model, Q, i, updateKin));// Forward kinematics
            updateKin = false;
        }
    return pos;
}
// Se faire renvoyer les IMUs anatomiques dans le repère local
std::vector<s2mIMU> s2mIMUs::anatomicalIMU(){
    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if ( IMU(i).isAnatomical() )
            pos.push_back(IMU(i));// Forward kinematics
    return pos;
}

std::vector<s2mIMU> s2mIMUs::segmentIMU(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const unsigned int &idx,
        const bool &updateKin){
    // Update de la cinématique
    if (updateKin)
        RigidBodyDynamics::UpdateKinematicsCustom(model, &Q,nullptr, nullptr);

    // Nom du segment a trouver
    biorbd::utils::String name(model.bone(idx).name());

    std::vector<s2mIMU> pos;
    for (unsigned int i=0; i<nIMUs(); ++i) // passer tous les markers et sélectionner les bons
        if (!(m_IMUs.begin()+i)->parent().compare(name))
            pos.push_back(IMU(model,Q,i,false));

    return pos;
}

// Se faire renvoyer la jacobienne des markers
std::vector<biorbd::utils::Matrix> s2mIMUs::IMUJacobian(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const bool &updateKin){
    return IMUJacobian(model, Q, updateKin, false);
}

// Se faire renvoyer la jacobienne des marker techniques
std::vector<biorbd::utils::Matrix> s2mIMUs::TechnicalIMUJacobian(
        s2mJoints& model,
        const biorbd::utils::GenCoord &Q,
        const bool &updateKin){
    return IMUJacobian(model, Q, updateKin, true);
}


// Protected function
std::vector<biorbd::utils::Matrix> s2mIMUs::IMUJacobian(
        s2mJoints &model,
        const biorbd::utils::GenCoord &Q,
        const bool &updateKin,
        bool lookForTechnical){
    std::vector<biorbd::utils::Matrix> G;

    bool first(true);
    for (unsigned int idx=0; idx<nIMUs(); ++idx){
        // Marqueur actuel
        s2mIMU node = IMU(idx);
        if (lookForTechnical && !node.isTechnical())
            continue;

        unsigned int id = model.GetBodyId(node.parent().c_str());
        biorbd::utils::Matrix G_tp(biorbd::utils::Matrix::Zero(9,model.dof_count));

        // Calcul de la jacobienne de ce Tag
        if (first)
            model.CalcMatRotJacobian(model, Q, id, node.rot(), G_tp, updateKin);
        else
            model.CalcMatRotJacobian(model, Q, id, node.rot(), G_tp, false); // False for speed

        G.push_back(G_tp);
        first = false;
    }

    return G;
}

unsigned int s2mIMUs::nTechIMUs(){
    unsigned int nTech = 0;
    if (nTech == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <s2mIMU>::iterator it = m_IMUs.begin(); it!=m_IMUs.end(); ++it)
            if ((*it).isTechnical())
                ++nTech;

    return nTech;
}

unsigned int s2mIMUs::nAnatIMUs(){
    unsigned int nAnat = 0;
    if (nAnat == 0) // Si la fonction n'a jamais été appelée encore
        for (std::vector <s2mIMU>::iterator it = m_IMUs.begin(); it!=m_IMUs.end(); ++it)
            if ((*it).isAnatomical())
                ++nAnat;

    return nAnat;
}

std::vector<biorbd::utils::String> s2mIMUs::IMUsNames(){
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nIMUs(); ++i)
        names.push_back(IMU(i).name());

    return names;
}

std::vector<biorbd::utils::String> s2mIMUs::technicalIMUsNames(){
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if (IMU(i).isTechnical())
            names.push_back(IMU(i).name());

    return names;
}

std::vector<biorbd::utils::String> s2mIMUs::anatomicalIMUsNames(){
    // Extrait le nom de tous les markers d'un modele
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nIMUs(); ++i)
        if (IMU(i).isAnatomical())
            names.push_back(IMU(i).name());

    return names;
}
