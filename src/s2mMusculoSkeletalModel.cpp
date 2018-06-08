#define BIORBD_API_EXPORTS
#include "../include/s2mMusculoSkeletalModel.h"

//s2mMusculoSkeletalModel::s2mMusculoSkeletalModel()
//{
//}

//s2mMusculoSkeletalModel::~s2mMusculoSkeletalModel()
//{
//    //dtor
//}

s2mMusculoSkeletalModel::s2mMusculoSkeletalModel()
{

}

s2mMusculoSkeletalModel::s2mMusculoSkeletalModel(const s2mPath &path)
{
    s2mRead::readModelFile(path, this);
}

void s2mMusculoSkeletalModel::coucou(s2mJoints &j)
{
    std::cout << j.nbQ() << std::endl;
}

bool s2mMusculoSkeletalModel::InverseKinematics(const std::vector<Eigen::Vector3d> &M, const s2mGenCoord &Qinit, s2mGenCoord &Q, bool removeAxes){
    // Trouver les markers techniques uniquement (body_point)
    std::vector<s2mNodeBone> body_point(this->TechnicalTagsInLocal(removeAxes));
    std::vector<RigidBodyDynamics::Math::Vector3d> body_pointEigen;
    for (std::vector<s2mNodeBone>::iterator it = body_point.begin(); it!=body_point.end(); ++it)
        body_pointEigen.push_back((*it).vector());

    std::vector<RigidBodyDynamics::Math::Vector3d> MRBDL;
    for (std::vector<Eigen::Vector3d>::const_iterator it = M.begin(); it!=M.end(); ++it)
        MRBDL.push_back(*it);

    // Associer le numéro de body a chaque marker technique (body_id)
    std::vector<unsigned int> body_id;
    for (unsigned int i=0; i<body_point.size(); ++i)
        body_id.push_back( (*(body_point.begin()+i)).parentId() );

    // Appeler la fonction de base
    return RigidBodyDynamics::InverseKinematics(*this, Qinit, body_id, body_pointEigen, MRBDL, Q);
}
