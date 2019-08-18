#define BIORBD_API_EXPORTS
#include "Muscles/PathChangers.h"

#include "Utils/Error.h"
#include "Muscles/PathChanger.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/WrappingSphere.h"
#include "Muscles/WrappingCylinder.h"

biorbd::muscles::PathChangers::PathChangers() :
    m_nbWraps(0),
    m_nbVia(0),
    m_totalObjects(0)
{

}

biorbd::muscles::PathChangers::~PathChangers(){

}

// Private method to assing values
void biorbd::muscles::PathChangers::addPathChanger(biorbd::muscles::PathChanger &val){

    // Ajouter un muscle au pool de muscle selon son type
    if (dynamic_cast<biorbd::muscles::WrappingSphere*> (&val)){
        biorbd::utils::Error::error(m_nbVia == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(std::shared_ptr<biorbd::muscles::PathChanger> (new biorbd::muscles::WrappingSphere(dynamic_cast <biorbd::muscles::WrappingSphere&> (val))));
        ++m_nbWraps;
    }
    else if (dynamic_cast<biorbd::muscles::WrappingCylinder*> (&val)){
        biorbd::utils::Error::error(m_nbVia == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(std::shared_ptr<biorbd::muscles::PathChanger> (new biorbd::muscles::WrappingCylinder(dynamic_cast <biorbd::muscles::WrappingCylinder&> (val))));
        ++m_nbWraps;
    }
    else if (dynamic_cast<biorbd::muscles::ViaPoint*> (&val)){
        biorbd::utils::Error::error(m_nbWraps == 0, "Cannot mix via points and wrapping objects yet");
        m_obj.push_back(std::shared_ptr<biorbd::muscles::PathChanger> (new biorbd::muscles::ViaPoint(dynamic_cast <biorbd::muscles::ViaPoint&> (val))));
        ++m_nbVia;
    }
    else
        biorbd::utils::Error::error(0, "Wrapping type not found");
    ++m_totalObjects;
}

unsigned int biorbd::muscles::PathChangers::nbWraps() const {
    return m_nbWraps;
}

unsigned int biorbd::muscles::PathChangers::nbVia() const
{
    return m_nbVia;
}

unsigned int biorbd::muscles::PathChangers::nbObjects() const
{
    return m_totalObjects;
}


const std::shared_ptr<biorbd::muscles::PathChanger> biorbd::muscles::PathChangers::object(unsigned int idx) const{
    biorbd::utils::Error::error(idx<nbObjects(), "Idx asked is higher than number of wrapping objects");
    return m_obj[idx];
}




