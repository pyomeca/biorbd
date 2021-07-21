#define BIORBD_API_EXPORTS
#include "Muscles/PathModifiers.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/WrappingSphere.h"
#include "Muscles/WrappingHalfCylinder.h"

using namespace BIORBD_NAMESPACE;

muscles::PathModifiers::PathModifiers() :
    m_obj(std::make_shared<std::vector<std::shared_ptr<utils::Vector3d>>>()),
    m_nbWraps(std::make_shared<unsigned int>(0)),
    m_nbVia(std::make_shared<unsigned int>(0)),
    m_totalObjects(std::make_shared<unsigned int>(0))
{

}

muscles::PathModifiers muscles::PathModifiers::DeepCopy() const
{
    muscles::PathModifiers copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::PathModifiers::DeepCopy(const
        muscles::PathModifiers &other)
{
    m_obj->resize(other.m_obj->size());
    for (unsigned int i=0; i<other.m_obj->size(); ++i) {
        (*m_obj)[i] = std::make_shared<utils::Vector3d>();
        *(*m_obj)[i] = (*other.m_obj)[i]->DeepCopy();
    }
    *m_nbWraps = *other.m_nbWraps;
    *m_nbVia = *other.m_nbVia;
    *m_totalObjects = *other.m_totalObjects;
}

// Private method to assing values
void muscles::PathModifiers::addPathChanger(
    utils::Vector3d &object)
{

    // Add a muscle to the pool of muscle depending on type
    if (object.typeOfNode() == utils::NODE_TYPE::WRAPPING_SPHERE) {
        utils::Error::check(*m_nbVia == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<muscles::WrappingSphere>(
                             static_cast<muscles::WrappingSphere&> (object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() ==
               utils::NODE_TYPE::WRAPPING_HALF_CYLINDER) {
        utils::Error::check(*m_nbVia == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<muscles::WrappingHalfCylinder>(
                             dynamic_cast <muscles::WrappingHalfCylinder&> (object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() == utils::NODE_TYPE::VIA_POINT) {
        utils::Error::check(*m_nbWraps == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<muscles::ViaPoint>(
                             dynamic_cast <muscles::ViaPoint&> (object)));
        ++*m_nbVia;
    } else {
        utils::Error::raise("Wrapping type not found");
    }
    ++*m_totalObjects;
}

unsigned int muscles::PathModifiers::nbWraps() const
{
    return *m_nbWraps;
}

unsigned int muscles::PathModifiers::nbVia() const
{
    return *m_nbVia;
}

unsigned int muscles::PathModifiers::nbObjects() const
{
    return *m_totalObjects;
}

utils::Vector3d& muscles::PathModifiers::object(
    unsigned int idx)
{
    utils::Error::check(idx<nbObjects(),
                                "Idx asked is higher than number of wrapping objects");
    return *(*m_obj)[idx];
}


const utils::Vector3d& muscles::PathModifiers::object(
    unsigned int idx) const
{
    utils::Error::check(idx<nbObjects(),
                                "Idx asked is higher than number of wrapping objects");
    return *(*m_obj)[idx];
}




