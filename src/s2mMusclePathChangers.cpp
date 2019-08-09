#define BIORBD_API_EXPORTS
#include "s2mMusclePathChangers.h"

#include "s2mError.h"
#include "s2mMusclePathChanger.h"
#include "s2mViaPoint.h"
#include "s2mWrappingSphere.h"
#include "s2mWrappingCylinder.h"

s2mMusclePathChangers::s2mMusclePathChangers() :
    m_nbWraps(0),
    m_nbVia(0),
    m_totalObjects(0)
{

}

s2mMusclePathChangers::~s2mMusclePathChangers(){

}

// Private method to assing values
void s2mMusclePathChangers::addPathChanger(s2mMusclePathChanger &val){

    // Ajouter un muscle au pool de muscle selon son type
    if (dynamic_cast<s2mWrappingSphere*> (&val)){
        s2mError::s2mAssert(m_nbVia == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(std::shared_ptr<s2mMusclePathChanger> (new s2mWrappingSphere(dynamic_cast <s2mWrappingSphere&> (val))));
        ++m_nbWraps;
    }
    else if (dynamic_cast<s2mWrappingCylinder*> (&val)){
        s2mError::s2mAssert(m_nbVia == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(std::shared_ptr<s2mMusclePathChanger> (new s2mWrappingCylinder(dynamic_cast <s2mWrappingCylinder&> (val))));
        ++m_nbWraps;
    }
    else if (dynamic_cast<s2mViaPoint*> (&val)){
        s2mError::s2mAssert(m_nbWraps == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(std::shared_ptr<s2mMusclePathChanger> (new s2mViaPoint(dynamic_cast <s2mViaPoint&> (val))));
        ++m_nbVia;
    }
    else
        s2mError::s2mAssert(0, "Wrapping type not found");
    ++m_totalObjects;
}

unsigned int s2mMusclePathChangers::nbWraps() const {
    return m_nbWraps;
}

unsigned int s2mMusclePathChangers::nbVia() const
{
    return m_nbVia;
}

unsigned int s2mMusclePathChangers::nbObjects() const
{
    return m_totalObjects;
}


const std::shared_ptr<s2mMusclePathChanger> s2mMusclePathChangers::object(const unsigned int &idx) const{
    s2mError::s2mAssert(idx<nbObjects(), "Idx asked is higher than number of wrapping objects");
    return m_obj[idx];
}




