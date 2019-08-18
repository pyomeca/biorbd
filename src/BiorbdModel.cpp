#define BIORBD_API_EXPORTS
#include "BiorbdModel.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "ModelReader.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/NodeBone.h"

biorbd::Model::Model()
{

}

biorbd::Model::~Model()
{

}

biorbd::Model::Model(const biorbd::utils::Path &path)
{
    biorbd::Reader::readModelFile(path, this);
}

bool biorbd::Model::InverseKinematics(
        const std::vector<biorbd::rigidbody::NodeBone> &markers,
        const biorbd::rigidbody::GeneralizedCoordinates &Qinit,
        biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool removeAxes){
    // Trouver les markers techniques uniquement (body_point)
    std::vector<biorbd::rigidbody::NodeBone> body_point(this->TechnicalTagsInLocal(removeAxes));
    std::vector<RigidBodyDynamics::Math::Vector3d> body_pointEigen;
    for (std::vector<biorbd::rigidbody::NodeBone>::iterator it = body_point.begin(); it!=body_point.end(); ++it)
        body_pointEigen.push_back((*it).vector());

    std::vector<RigidBodyDynamics::Math::Vector3d> markersInRbdl;
    for (unsigned int i = 0; i<markers.size(); ++i)
        markersInRbdl.push_back(markers[i]);

    // Associer le numéro de body a chaque marker technique (body_id)
    std::vector<unsigned int> body_id;
    for (unsigned int i=0; i<body_point.size(); ++i)
        body_id.push_back( static_cast<unsigned int>((*(body_point.begin()+i)).parentId()) );

    // Appeler la fonction de base
    return RigidBodyDynamics::InverseKinematics(*this, Qinit, body_id, body_pointEigen, markersInRbdl, Q);
}
