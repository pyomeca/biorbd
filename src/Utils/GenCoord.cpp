#define BIORBD_API_EXPORTS
#include "Utils/GenCoord.h"

#include "s2mJoints.h"

biorbd::utils::GenCoord::GenCoord()
{

}

biorbd::utils::GenCoord::GenCoord(const GenCoord &Q):
    biorbd::utils::Vector(Q)
{

}

biorbd::utils::GenCoord::GenCoord(const biorbd::utils::Vector &v) :
    biorbd::utils::Vector(v)
{

}

biorbd::utils::GenCoord::GenCoord(const Eigen::VectorXd &v) :
    biorbd::utils::Vector(v)
{

}

biorbd::utils::GenCoord::GenCoord(unsigned int i) : biorbd::utils::Vector(i)
{

}

biorbd::utils::GenCoord::GenCoord(const s2mJoints &j) :
    biorbd::utils::Vector(j.nbDof()){

}

biorbd::utils::GenCoord::~GenCoord()
{

}

biorbd::utils::GenCoord& biorbd::utils::GenCoord::operator=(const Eigen::VectorXd& vecX){
    if (this==&vecX) // check for self-assigment
        return *this;

    this->block(0,0,this->rows(),1) = vecX;

    return *this;
}
