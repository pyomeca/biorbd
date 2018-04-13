#include "../include/s2mMusclePathChangers.h"



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
        m_obj.push_back(boost::shared_ptr<s2mMusclePathChanger> (new s2mWrappingSphere(dynamic_cast <s2mWrappingSphere&> (val))));
        ++m_nbWraps;
    }
    else if (dynamic_cast<s2mWrappingCylinder*> (&val)){
        s2mError::s2mAssert(m_nbVia == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(boost::shared_ptr<s2mMusclePathChanger> (new s2mWrappingCylinder(dynamic_cast <s2mWrappingCylinder&> (val))));
        ++m_nbWraps;
    }
    else if (dynamic_cast<s2mViaPoint*> (&val)){
        s2mError::s2mAssert(m_nbWraps == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(boost::shared_ptr<s2mMusclePathChanger> (new s2mViaPoint(dynamic_cast <s2mViaPoint&> (val))));
        ++m_nbVia;
    }
    else
        s2mError::s2mAssert(0, "Wrapping type not found");
    ++m_totalObjects;
}


boost::shared_ptr<s2mMusclePathChanger> s2mMusclePathChangers:: object(const unsigned int &idx) const{
    s2mError::s2mAssert(idx<nbObjects(), "Idx asked is higher than number of wrapping objects");
    return *(m_obj.begin() + idx);
}




