#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/MuscleGroup.h"

#include "Utils/Error.h"
#include "InternalForces/Muscles/IdealizedActuator.h"
#include "InternalForces/Muscles/HillType.h"
#include "InternalForces/Muscles/HillThelenType.h"
#include "InternalForces/Muscles/HillDeGrooteType.h"
#include "InternalForces/Muscles/HillThelenActiveOnlyType.h"
#include "InternalForces/Muscles/HillThelenTypeFatigable.h"
#include "InternalForces/Muscles/HillDeGrooteActiveOnlyType.h"
#include "InternalForces/Muscles/HillDeGrooteTypeFatigable.h"
#include "InternalForces/Muscles/StateDynamicsBuchanan.h"
#include "InternalForces/Muscles/StateDynamicsDeGroote.h"

using namespace BIORBD_NAMESPACE;
internal_forces::muscles::MuscleGroup::MuscleGroup() :
    m_mus(std::make_shared<std::vector<std::shared_ptr<internal_forces::muscles::Muscle>>>()),
    m_name(std::make_shared<utils::String>()),
    m_originName(std::make_shared<utils::String>()),
    m_insertName(std::make_shared<utils::String>())
{

}

internal_forces::muscles::MuscleGroup::MuscleGroup(const internal_forces::muscles::MuscleGroup
        &other) :
    m_mus(other.m_mus),
    m_name(other.m_name),
    m_originName(other.m_originName),
    m_insertName(other.m_insertName)
{

}

internal_forces::muscles::MuscleGroup::MuscleGroup(
    const utils::String &name,
    const utils::String &originName,
    const utils::String &insertionName) :
    m_mus(std::make_shared<std::vector<std::shared_ptr<internal_forces::muscles::Muscle>>>()),
    m_name(std::make_shared<utils::String>(name)),
    m_originName(std::make_shared<utils::String>(originName)),
    m_insertName(std::make_shared<utils::String>(insertionName))
{
}

internal_forces::muscles::MuscleGroup::~MuscleGroup()
{

}

internal_forces::muscles::MuscleGroup internal_forces::muscles::MuscleGroup::DeepCopy() const
{
    internal_forces::muscles::MuscleGroup copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::MuscleGroup::DeepCopy(
        const internal_forces::muscles::MuscleGroup &other)
{
    m_mus->resize(other.m_mus->size());
    for (size_t i=0; i<other.m_mus->size(); ++i) {
        if ((*other.m_mus)[i]->type() ==
                internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::IdealizedActuator>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() == internal_forces::muscles::MUSCLE_TYPE::HILL) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::HillType>((*other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::HillThelenType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::HillDeGrooteType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::HillThelenActiveOnlyType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::HillThelenTypeFatigable>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::HillDeGrooteActiveOnlyType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE) {
            (*m_mus)[i] = std::make_shared<internal_forces::muscles::HillDeGrooteTypeFatigable>((
                              *other.m_mus)[i]);
        } else {
            utils::Error::raise("DeepCopy was not prepared to copy " +
                                        utils::String(
                                            internal_forces::muscles::MUSCLE_TYPE_toStr((*other.m_mus)[i]->type())) + " type");
        }
    }
    *m_mus = *other.m_mus;
    *m_name = *other.m_name;
    *m_originName = *other.m_originName;
    *m_insertName = *other.m_insertName;
}

void internal_forces::muscles::MuscleGroup::addMuscle(
    const utils::String &name,
    internal_forces::muscles::MUSCLE_TYPE type,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    internal_forces::muscles::STATE_TYPE stateType,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<internal_forces::muscles::Muscle> muscle;
    std::shared_ptr<internal_forces::muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == internal_forces::muscles::STATE_TYPE::SIMPLE_STATE) {
        state = std::make_shared<internal_forces::muscles::StateDynamics>();
    } else if (stateType == internal_forces::muscles::STATE_TYPE::DYNAMIC) {
        state = std::make_shared<internal_forces::muscles::StateDynamics>();
    } else if (stateType == internal_forces::muscles::STATE_TYPE::BUCHANAN) {
        state = std::make_shared<internal_forces::muscles::StateDynamicsBuchanan>();
    } else if (stateType == internal_forces::muscles::STATE_TYPE::DE_GROOTE) {
        state = std::make_shared<internal_forces::muscles::StateDynamicsDeGroote>();
    } else {
        state = std::make_shared<internal_forces::muscles::StateDynamics>();
    }

    if (type == internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<internal_forces::muscles::IdealizedActuator>(name,geometry,
                 characteristics, *state);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<internal_forces::muscles::HillType>(name,geometry,
                 characteristics, *state);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteType>(name,geometry,
                 characteristics, *state);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenType>(name,geometry,
                 characteristics, *state);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics, *state);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics, *state,
                 dynamicFatigueType);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteActiveOnlyType>(name,
                 geometry,characteristics, *state);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteTypeFatigable>(name,
                 geometry,characteristics, *state,
                 dynamicFatigueType);
    } else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void internal_forces::muscles::MuscleGroup::addMuscle(
    const utils::String &name,
    internal_forces::muscles::MUSCLE_TYPE type,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<internal_forces::muscles::Muscle> muscle;
    if (type == internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<internal_forces::muscles::IdealizedActuator>(name,geometry,
                 characteristics);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<internal_forces::muscles::HillType>(name,geometry,
                 characteristics);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteType>(name,geometry,
                 characteristics);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenType>(name,geometry,
                 characteristics);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics, dynamicFatigueType);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteActiveOnlyType>(name,
                 geometry,characteristics);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteTypeFatigable>(name,
                 geometry,characteristics, dynamicFatigueType);
    } else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void internal_forces::muscles::MuscleGroup::addMuscle(
    const utils::String& name,
    internal_forces::muscles::MUSCLE_TYPE type,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    internal_forces::muscles::STATE_TYPE stateType,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<internal_forces::muscles::Muscle> muscle;
    std::shared_ptr<internal_forces::muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == internal_forces::muscles::STATE_TYPE::SIMPLE_STATE) {
        state = std::make_shared<internal_forces::muscles::StateDynamics>();
    } else if (stateType == internal_forces::muscles::STATE_TYPE::BUCHANAN) {
        state = std::make_shared<internal_forces::muscles::StateDynamicsBuchanan>();
    } else if (stateType == internal_forces::muscles::STATE_TYPE::DE_GROOTE) {
        state = std::make_shared<internal_forces::muscles::StateDynamicsDeGroote>();
    } else {
        state = std::make_shared<internal_forces::muscles::StateDynamics>();
    }


    if (type == internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        muscle = std::make_shared<internal_forces::muscles::IdealizedActuator>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL)
        muscle = std::make_shared<internal_forces::muscles::HillType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE)
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN)
        muscle = std::make_shared<internal_forces::muscles::HillThelenType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE)
        muscle = std::make_shared<internal_forces::muscles::HillThelenActiveOnlyType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        muscle = std::make_shared<internal_forces::muscles::HillThelenTypeFatigable>(
                     name,geometry,characteristics,pathModifiers,*state,dynamicFatigueType);
    else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE)
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteActiveOnlyType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE)
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteTypeFatigable>(
                     name,geometry,characteristics,pathModifiers,*state,dynamicFatigueType);
    else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void internal_forces::muscles::MuscleGroup::addMuscle(
    const utils::String &name,
    internal_forces::muscles::MUSCLE_TYPE type,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<internal_forces::muscles::Muscle> muscle;

    if (type == internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<internal_forces::muscles::IdealizedActuator>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<internal_forces::muscles::HillType>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteType>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenType>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics,pathModifiers);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<internal_forces::muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics,pathModifiers,
                 dynamicFatigueType);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteActiveOnlyType>(name,
                 geometry,characteristics,pathModifiers);
    } else if (type == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE) {
        muscle = std::make_shared<internal_forces::muscles::HillDeGrooteTypeFatigable>(name,
                 geometry,characteristics,pathModifiers,
                 dynamicFatigueType);
    } else {
        utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void internal_forces::muscles::MuscleGroup::addMuscle(
    const internal_forces::muscles::Muscle &muscle)
{
    utils::Error::check(muscleID(muscle.name()) == -1,
                                "This muscle name was already defined for this muscle group");

    // Add muscle according to its type
    if (muscle.type() == internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::IdealizedActuator>(muscle));
    } else if (muscle.type() ==
               internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::HillThelenTypeFatigable>
                         (muscle));
    } else if (muscle.type() ==
               internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::HillDeGrooteTypeFatigable>
                         (muscle));
    } else if (muscle.type() == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::HillThelenType>(muscle));
    } else if (muscle.type() == internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::HillThelenActiveOnlyType>
                         (muscle));
    } else if (muscle.type() == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::HillDeGrooteActiveOnlyType>
                         (muscle));
    } else if (muscle.type() == internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::HillDeGrooteType>(muscle));
    } else if (muscle.type() == internal_forces::muscles::MUSCLE_TYPE::HILL) {
        m_mus->push_back(std::make_shared<internal_forces::muscles::HillType>(muscle));
    } else {
        utils::Error::raise("Muscle type not found");
    }
    return;
}

internal_forces::muscles::Muscle& internal_forces::muscles::MuscleGroup::muscle(size_t idx)
{
    utils::Error::check(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}
const internal_forces::muscles::Muscle& internal_forces::muscles::MuscleGroup::muscle(
    size_t idx) const
{
    utils::Error::check(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}


size_t internal_forces::muscles::MuscleGroup::nbMuscles() const
{
    return m_mus->size();
}

std::vector<std::shared_ptr<internal_forces::muscles::Muscle>>&
        internal_forces::muscles::MuscleGroup::muscles()
{
    return *m_mus;
}

const std::vector<std::shared_ptr<internal_forces::muscles::Muscle>>&
        internal_forces::muscles::MuscleGroup::muscles() const
{
    return *m_mus;
}

int internal_forces::muscles::MuscleGroup::muscleID(const utils::String&
        nameToFind)
{
    for (size_t i=0; i<m_mus->size(); ++i) {
        if (!nameToFind.compare( (*m_mus)[i]->name()) ) {
            return static_cast<int>(i);
        }
    }
    // If we get here, there is no muscle of that name in this group
    return -1;
}

void internal_forces::muscles::MuscleGroup::setName(const utils::String& name)
{
    *m_name = name;
}
const utils::String &internal_forces::muscles::MuscleGroup::name() const
{
    return *m_name;
}

void internal_forces::muscles::MuscleGroup::setOrigin(const utils::String& name)
{
    *m_originName = name;
}
const utils::String &internal_forces::muscles::MuscleGroup::origin() const
{
    return *m_originName;
}

void internal_forces::muscles::MuscleGroup::setInsertion(const utils::String&
        name)
{
    *m_insertName = name;
}
const utils::String &internal_forces::muscles::MuscleGroup::insertion() const
{
    return *m_insertName;
}
