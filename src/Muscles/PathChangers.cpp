#define BIORBD_API_EXPORTS
#include "Muscles/PathChangers.h"

#include "Utils/Error.h"
#include "Utils/Node3d.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/WrappingSphere.h"
#include "Muscles/WrappingCylinder.h"

biorbd::muscles::PathChangers::PathChangers() :
    m_obj(std::make_shared<std::vector<biorbd::utils::Node3d>>()),
    m_nbWraps(std::make_shared<unsigned int>(0)),
    m_nbVia(std::make_shared<unsigned int>(0)),
    m_totalObjects(std::make_shared<unsigned int>(0))
{

}

biorbd::muscles::PathChangers biorbd::muscles::PathChangers::DeepCopy() const
{
    biorbd::muscles::PathChangers copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::PathChangers::DeepCopy(const biorbd::muscles::PathChangers &other)
{
    m_obj->resize(other.m_obj->size());
    for (unsigned int i=0; i<other.m_obj->size(); ++i)
        (*m_obj)[i] = (*other.m_obj)[i].DeepCopy();
    *m_nbWraps = *other.m_nbWraps;
    *m_nbVia = *other.m_nbVia;
    *m_totalObjects = *other.m_totalObjects;
}

// Private method to assing values
void biorbd::muscles::PathChangers::addPathChanger(biorbd::utils::Node3d &val){

    // Ajouter un muscle au pool de muscle selon son type
    if (dynamic_cast<biorbd::muscles::WrappingSphere*> (&val)){
        biorbd::utils::Error::error(*m_nbVia == 0, "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(biorbd::muscles::WrappingSphere(static_cast<biorbd::muscles::WrappingSphere&> (val)));
        ++*m_nbWraps;
    }
    else if (dynamic_cast<biorbd::muscles::WrappingCylinder*> (&val)){
        biorbd::utils::Error::error(*m_nbVia == 0, "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(biorbd::muscles::WrappingCylinder(dynamic_cast <biorbd::muscles::WrappingCylinder&> (val)));
        ++*m_nbWraps;
    }
    else if (dynamic_cast<biorbd::muscles::ViaPoint*> (&val)){
        biorbd::utils::Error::error(*m_nbWraps == 0, "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(biorbd::muscles::ViaPoint(dynamic_cast <biorbd::muscles::ViaPoint&> (val)));
        ++*m_nbVia;
    }
    else
        biorbd::utils::Error::error(0, "Wrapping type not found");
    ++*m_totalObjects;
}

unsigned int biorbd::muscles::PathChangers::nbWraps() const {
    return *m_nbWraps;
}

unsigned int biorbd::muscles::PathChangers::nbVia() const
{
    return *m_nbVia;
}

unsigned int biorbd::muscles::PathChangers::nbObjects() const
{
    return *m_totalObjects;
}

biorbd::utils::Node3d &biorbd::muscles::PathChangers::object(unsigned int idx)
{
    biorbd::utils::Error::error(idx<nbObjects(), "Idx asked is higher than number of wrapping objects");
    return (*m_obj)[idx];
}


const biorbd::utils::Node3d& biorbd::muscles::PathChangers::object(unsigned int idx) const{
    biorbd::utils::Error::error(idx<nbObjects(), "Idx asked is higher than number of wrapping objects");
    return (*m_obj)[idx];
}




