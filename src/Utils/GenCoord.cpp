#define BIORBD_API_EXPORTS
#include "Utils/GenCoord.h"

#include "s2mJoints.h"

namespace biorbd { namespace utils {

GenCoord::GenCoord() {}

GenCoord::GenCoord(const GenCoord &Q):
    s2mVector(Q)
{

}

GenCoord::GenCoord(const s2mVector &v) : s2mVector(v) {}

GenCoord::GenCoord(const Eigen::VectorXd &v) : s2mVector(v) {}

GenCoord::GenCoord(unsigned int i) : s2mVector(i) {}

GenCoord::GenCoord(const s2mJoints &j) :
    s2mVector(j.nbDof()){

}

GenCoord::~GenCoord()
{

}

GenCoord& GenCoord::operator=(const Eigen::VectorXd& vecX){
    if (this==&vecX) // check for self-assigment
        return *this;

    this->block(0,0,this->rows(),1) = vecX;

    return *this;
}

}}
