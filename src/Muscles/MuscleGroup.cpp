#define BIORBD_API_EXPORTS
#include "Muscles/MuscleGroup.h"

#include "Utils/Error.h"
#include "Muscles/IdealizedActuator.h"
#include "Muscles/HillType.h"
#include "Muscles/HillThelenType.h"
#include "Muscles/HillThelenActiveOnlyType.h"
#include "Muscles/HillThelenTypeFatigable.h"
#include "Muscles/StateDynamicsBuchanan.h"
#include "Muscles/StateDynamicsDeGroote.h"

using namespace BIORBD_NAMESPACE;

muscles::MuscleGroup::MuscleGroup() :
    m_mus(std::make_shared<std::vector<std::shared_ptr<muscles::Muscle>>>()),
    m_name(std::make_shared<utils::String>()),
    m_originName(std::make_shared<utils::String>()),
    m_insertName(std::make_shared<utils::String>())
{

}

muscles::MuscleGroup::MuscleGroup(const muscles::MuscleGroup
        &other) :
    m_mus(other.m_mus),
    m_name(other.m_name),
    m_originName(other.m_originName),
    m_insertName(other.m_insertName)
{

}

muscles::MuscleGroup::MuscleGroup(
    const utils::String &name,
    const utils::String &originName,
    const utils::String &insertionName) :
    m_mus(std::make_shared<std::vector<std::shared_ptr<muscles::Muscle>>>()),
    m_name(std::make_shared<utils::String>(name)),
    m_originName(std::make_shared<utils::String>(originName)),
    m_insertName(std::make_shared<utils::String>(insertionName))
{
}

muscles::MuscleGroup::~MuscleGroup()
{

}

muscles::MuscleGroup muscles::MuscleGroup::DeepCopy() const
{
    muscles::MuscleGroup copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::MuscleGroup::DeepCopy(const muscles::MuscleGroup
        &other)
{
    m_mus->resize(other.m_mus->size());
    for (unsigned int i=0; i<other.m_mus->size(); ++i) {
        if ((*other.m_mus)[i]->type() ==
                muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
            (*m_mus)[i] = std::make_shared<muscles::IdealizedActuator>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() == muscles::MUSCLE_TYPE::HILL) {
            (*m_mus)[i] = std::make_shared<muscles::HillType>((*other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   muscles::MUSCLE_TYPE::HILL_THELEN) {
            (*m_mus)[i] = std::make_shared<muscles::HillThelenType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
            (*m_mus)[i] = std::make_shared<muscles::HillThelenActiveOnlyType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
            (*m_mus)[i] = std::make_shared<muscles::HillThelenTypeFatigable>((
                              *other.m_mus)[i]);
        } else {
            utils::Error::raise("DeepCopy was not prepared to copy " +
                                        utils::String(
                                            muscles::MUSCLE_TYPE_toStr((*other.m_mus)[i]->type())) + " type");
        }
    }
    *m_mus = *other.m_mus;
    *m_name = *other.m_name;
    *m_originName = *other.m_originName;
    *m_insertName = *other.m_insertName;
}

void muscles::MuscleGroup::addMuscle(
    const utils::String &name,
    muscles::MUSCLE_TYPE type,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    muscles::STATE_TYPE stateType,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<muscles::Muscle> muscle;
    std::shared_ptr<muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == muscles::STATE_TYPE::SIMPLE_STATE) {
        state = std::make_shared<muscles::StateDynamics>();
    } else if (stateType == muscles::STATE_TYPE::DYNAMIC) {
        state = std::make_shared<muscles::StateDynamics>();
    } else if (stateType == muscles::STATE_TYPE::BUCHANAN) {
        state = std::make_shared<muscles::StateDynamicsBuchanan>();
    } else if (stateType == muscles::STATE_TYPE::DE_GROOTE) {
        state = std::make_shared<muscles::StateDynamicsDeGroote>();
    } else {
        state = std::make_shared<muscles::StateDynamics>();
    }

    if (type == muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<muscles::IdealizedActuator>(name,geometry,
                 characteristics, *state);
    } else if (type == muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<muscles::HillType>(name,geometry,
                 characteristics, *state);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<muscles::HillThelenType>(name,geometry,
                 characteristics, *state);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics, *state);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics, *state,
                 dynamicFatigueType);
    } else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void muscles::MuscleGroup::addMuscle(
    const utils::String &name,
    muscles::MUSCLE_TYPE type,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<muscles::Muscle> muscle;
    if (type == muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<muscles::IdealizedActuator>(name,geometry,
                 characteristics);
    } else if (type == muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<muscles::HillType>(name,geometry,
                 characteristics);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<muscles::HillThelenType>(name,geometry,
                 characteristics);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics, dynamicFatigueType);
    } else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void muscles::MuscleGroup::addMuscle(
    const utils::String& name,
    muscles::MUSCLE_TYPE type,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers &pathModifiers,
    muscles::STATE_TYPE stateType,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<muscles::Muscle> muscle;
    std::shared_ptr<muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == muscles::STATE_TYPE::SIMPLE_STATE) {
        state = std::make_shared<muscles::StateDynamics>();
    } else if (stateType == muscles::STATE_TYPE::BUCHANAN) {
        state = std::make_shared<muscles::StateDynamicsBuchanan>();
    } else if (stateType == muscles::STATE_TYPE::DE_GROOTE) {
        state = std::make_shared<muscles::StateDynamicsDeGroote>();
    } else {
        state = std::make_shared<muscles::StateDynamics>();
    }


    if (type == muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        muscle = std::make_shared<muscles::IdealizedActuator>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == muscles::MUSCLE_TYPE::HILL)
        muscle = std::make_shared<muscles::HillType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == muscles::MUSCLE_TYPE::HILL_THELEN)
        muscle = std::make_shared<muscles::HillThelenType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE)
        muscle = std::make_shared<muscles::HillThelenActiveOnlyType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        muscle = std::make_shared<muscles::HillThelenTypeFatigable>(
                     name,geometry,characteristics,pathModifiers,*state,dynamicFatigueType);
    else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void muscles::MuscleGroup::addMuscle(
    const utils::String &name,
    muscles::MUSCLE_TYPE type,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<muscles::Muscle> muscle;

    if (type == muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<muscles::IdealizedActuator>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<muscles::HillType>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<muscles::HillThelenType>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics,pathModifiers);
    } else if (type == muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics,pathModifiers,
                 dynamicFatigueType);
    } else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void muscles::MuscleGroup::addMuscle(
    const muscles::Muscle &muscle)
{
    utils::Error::check(muscleID(muscle.name()) == -1,
                                "This muscle name was already defined for this muscle group");

    // Add muscle according to its type
    if (muscle.type() == muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        m_mus->push_back(std::make_shared<muscles::IdealizedActuator>(muscle));
    } else if (muscle.type() ==
               muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        m_mus->push_back(std::make_shared<muscles::HillThelenTypeFatigable>
                         (muscle));
    } else if (muscle.type() == muscles::MUSCLE_TYPE::HILL_THELEN) {
        m_mus->push_back(std::make_shared<muscles::HillThelenType>(muscle));
    } else if (muscle.type() == muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        m_mus->push_back(std::make_shared<muscles::HillThelenActiveOnlyType>
                         (muscle));
    } else if (muscle.type() == muscles::MUSCLE_TYPE::HILL) {
        m_mus->push_back(std::make_shared<muscles::HillType>(muscle));
    } else {
        utils::Error::raise("Muscle type not found");
    }
    return;
}

muscles::Muscle& muscles::MuscleGroup::muscle(unsigned int idx)
{
    utils::Error::check(idx<nbMuscles(),
                                "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}
const muscles::Muscle& muscles::MuscleGroup::muscle(
    unsigned int idx) const
{
    utils::Error::check(idx<nbMuscles(),
                                "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}


unsigned int muscles::MuscleGroup::nbMuscles() const
{
    return static_cast<unsigned int>(m_mus->size());
}

std::vector<std::shared_ptr<muscles::Muscle>>&
        muscles::MuscleGroup::muscles()
{
    return *m_mus;
}

const std::vector<std::shared_ptr<muscles::Muscle>>&
        muscles::MuscleGroup::muscles() const
{
    return *m_mus;
}

int muscles::MuscleGroup::muscleID(const utils::String&
        nameToFind)
{
    for (unsigned int i=0; i<m_mus->size(); ++i) {
        if (!nameToFind.compare( (*m_mus)[i]->name()) ) {
            return static_cast<int>(i);
        }
    }
    // If we get here, there is no muscle of that name in this group
    return -1;
}

void muscles::MuscleGroup::setName(const utils::String& name)
{
    *m_name = name;
}
const utils::String &muscles::MuscleGroup::name() const
{
    return *m_name;
}

void muscles::MuscleGroup::setOrigin(const utils::String& name)
{
    *m_originName = name;
}
const utils::String &muscles::MuscleGroup::origin() const
{
    return *m_originName;
}

void muscles::MuscleGroup::setInsertion(const utils::String&
        name)
{
    *m_insertName = name;
}
const utils::String &muscles::MuscleGroup::insertion() const
{
    return *m_insertName;
}
