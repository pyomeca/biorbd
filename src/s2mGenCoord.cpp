#define BIORBD_API_EXPORTS
#include "s2mGenCoord.h"

#include "s2mJoints.h"

s2mGenCoord::s2mGenCoord() {}

s2mGenCoord::s2mGenCoord(const s2mGenCoord &Q):
    s2mVector(Q)
{

}

s2mGenCoord::s2mGenCoord(const s2mVector &v) : s2mVector(v) {}

s2mGenCoord::s2mGenCoord(const Eigen::VectorXd &v) : s2mVector(v) {}

s2mGenCoord::s2mGenCoord(unsigned int i) : s2mVector(i) {}

s2mGenCoord::s2mGenCoord(const s2mJoints &j) :
    s2mVector(j.nbDof()){

}

s2mGenCoord& s2mGenCoord::operator=(const Eigen::VectorXd& vecX){
    if (this==&vecX) // check for self-assigment
        return *this;

    this->block(0,0,this->rows(),1) = vecX;

    return *this;
}
