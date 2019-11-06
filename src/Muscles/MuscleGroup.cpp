#define BIORBD_API_EXPORTS
#include "Muscles/MuscleGroup.h"

#include "Utils/Error.h"
#include "Muscles/IdealizedActuator.h"
#include "Muscles/HillType.h"
#include "Muscles/HillThelenType.h"
#include "Muscles/HillThelenTypeFatigable.h"
#include "Muscles/StateDynamicsBuchanan.h"

biorbd::muscles::MuscleGroup::MuscleGroup() :
    m_mus(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::Muscle>>>()),
    m_name(std::make_shared<biorbd::utils::String>()),
    m_originName(std::make_shared<biorbd::utils::String>()),
    m_insertName(std::make_shared<biorbd::utils::String>())
{

}

biorbd::muscles::MuscleGroup::MuscleGroup(const biorbd::muscles::MuscleGroup &other) :
    m_mus(other.m_mus),
    m_name(other.m_name),
    m_originName(other.m_originName),
    m_insertName(other.m_insertName)
{

}

biorbd::muscles::MuscleGroup::MuscleGroup(
        const biorbd::utils::String &name,
        const biorbd::utils::String &originName,
        const biorbd::utils::String &insertionName) :
    m_mus(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::Muscle>>>()),
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_originName(std::make_shared<biorbd::utils::String>(originName)),
    m_insertName(std::make_shared<biorbd::utils::String>(insertionName))
{
}

biorbd::muscles::MuscleGroup::~MuscleGroup()
{

}

biorbd::muscles::MuscleGroup biorbd::muscles::MuscleGroup::DeepCopy() const
{
    biorbd::muscles::MuscleGroup copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::MuscleGroup::DeepCopy(const biorbd::muscles::MuscleGroup &other)
{
    m_mus->resize(other.m_mus->size());
    for (unsigned int i=0; i<other.m_mus->size(); ++i){
        if ((*other.m_mus)[i]->type() == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
            (*m_mus)[i] = std::make_shared<biorbd::muscles::IdealizedActuator>((*other.m_mus)[i]);
        else if ((*other.m_mus)[i]->type() == biorbd::muscles::MUSCLE_TYPE::HILL)
            (*m_mus)[i] = std::make_shared<biorbd::muscles::HillType>((*other.m_mus)[i]);
        else if ((*other.m_mus)[i]->type() == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN)
            (*m_mus)[i] = std::make_shared<biorbd::muscles::HillThelenType>((*other.m_mus)[i]);
        else if ((*other.m_mus)[i]->type() == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
            (*m_mus)[i] = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>((*other.m_mus)[i]);
        else
            biorbd::utils::Error::raise("DeepCopy was not prepared to copy " + biorbd::utils::String(biorbd::muscles::MUSCLE_TYPE_toStr((*other.m_mus)[i]->type())) + " type");
    }
    *m_mus = *other.m_mus;
    *m_name = *other.m_name;
    *m_originName = *other.m_originName;
    *m_insertName = *other.m_insertName;
}

void biorbd::muscles::MuscleGroup::addMuscle(
        const biorbd::utils::String &name,
        biorbd::muscles::MUSCLE_TYPE type,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        biorbd::muscles::STATE_TYPE stateType,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;
    std::shared_ptr<biorbd::muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == biorbd::muscles::STATE_TYPE::SIMPLE_STATE)
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    else if (stateType == biorbd::muscles::STATE_TYPE::DYNAMIC)
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    else if (stateType == biorbd::muscles::STATE_TYPE::BUCHANAN)
        state = std::make_shared<biorbd::muscles::StateDynamicsBuchanan>();
    else
        state = std::make_shared<biorbd::muscles::StateDynamics>();

    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>(name,geometry,characteristics, *state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL)
        muscle = std::make_shared<biorbd::muscles::HillType>(name,geometry,characteristics, *state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN)
        muscle = std::make_shared<biorbd::muscles::HillThelenType>(name,geometry,characteristics, *state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(name,geometry,characteristics, *state, dynamicFatigueType);
    else
        biorbd::utils::Error::raise("Wrong muscle type");
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
        const biorbd::utils::String &name,
        biorbd::muscles::MUSCLE_TYPE type,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;
    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>(name,geometry,characteristics);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL)
        muscle = std::make_shared<biorbd::muscles::HillType>(name,geometry,characteristics);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN)
        muscle = std::make_shared<biorbd::muscles::HillThelenType>(name,geometry,characteristics);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(name,geometry,characteristics, dynamicFatigueType);
    else
        biorbd::utils::Error::raise("Wrong muscle type");
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
        const biorbd::utils::String& name,
        biorbd::muscles::MUSCLE_TYPE type,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::PathChangers& pathChangers,
        biorbd::muscles::STATE_TYPE stateType,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;
    std::shared_ptr<biorbd::muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == biorbd::muscles::STATE_TYPE::SIMPLE_STATE)
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    else if (stateType == biorbd::muscles::STATE_TYPE::BUCHANAN)
        state = std::make_shared<biorbd::muscles::StateDynamicsBuchanan>();


    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>
                (name,geometry,characteristics,pathChangers,*state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL)
        muscle = std::make_shared<biorbd::muscles::HillType>
                (name,geometry,characteristics,pathChangers,*state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN)
        muscle = std::make_shared<biorbd::muscles::HillThelenType>
                (name,geometry,characteristics,pathChangers,*state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(
                    name,geometry,characteristics,pathChangers,*state,dynamicFatigueType);
    else
        biorbd::utils::Error::raise("Wrong muscle type");
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
        const biorbd::utils::String &name,
        biorbd::muscles::MUSCLE_TYPE type,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        const biorbd::muscles::PathChangers &pathChangers,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;

    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>(name,geometry,characteristics,pathChangers);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL)
        muscle = std::make_shared<biorbd::muscles::HillType>(name,geometry,characteristics,pathChangers);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN)
        muscle = std::make_shared<biorbd::muscles::HillThelenType>(name,geometry,characteristics,pathChangers);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(name,geometry,characteristics,pathChangers,dynamicFatigueType);
    else
        biorbd::utils::Error::raise("Wrong muscle type");
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
        const biorbd::muscles::Muscle &val)
{
    biorbd::utils::Error::check(muscleID(val.name()) == -1, "This muscle name was already defined for this muscle group");

    // Add muscle according to its type
    if (val.type() == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        m_mus->push_back(std::make_shared<biorbd::muscles::IdealizedActuator>(val));
    else if (val.type() == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        m_mus->push_back(std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(val));
    else if (val.type() == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN)
        m_mus->push_back(std::make_shared<biorbd::muscles::HillThelenType>(val));
    else if (val.type() == biorbd::muscles::MUSCLE_TYPE::HILL)
        m_mus->push_back(std::make_shared<biorbd::muscles::HillType>(val));
    else
        biorbd::utils::Error::raise("Muscle type not found");
    return;
}

biorbd::muscles::Muscle& biorbd::muscles::MuscleGroup::muscle(unsigned int idx)
{
    biorbd::utils::Error::check(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}
const biorbd::muscles::Muscle& biorbd::muscles::MuscleGroup::muscle(unsigned int idx) const
{
    biorbd::utils::Error::check(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}


unsigned int biorbd::muscles::MuscleGroup::nbMuscles() const {
    return static_cast<unsigned int>(m_mus->size());
}


int biorbd::muscles::MuscleGroup::muscleID(const biorbd::utils::String& nameToFind){
    for (unsigned int i=0; i<m_mus->size(); ++i){
       if (!nameToFind.compare( (*m_mus)[i]->name()) )
           return static_cast<int>(i);
    }
    // Si on se rend ici, c'est qu'il n'y a pas de muscle de ce nom dans le groupe
    return -1;
}

void biorbd::muscles::MuscleGroup::setName(const biorbd::utils::String& name) {
    *m_name = name;
}

void biorbd::muscles::MuscleGroup::setOrigin(const biorbd::utils::String& name) {
    *m_originName = name;
}

void biorbd::muscles::MuscleGroup::setInsertion(const biorbd::utils::String& name) {
    *m_insertName = name;
}

const biorbd::utils::String &biorbd::muscles::MuscleGroup::name() const {
    return *m_name;
}

const biorbd::utils::String &biorbd::muscles::MuscleGroup::origin() const {
    return *m_originName;
}

const biorbd::utils::String &biorbd::muscles::MuscleGroup::insertion() const {
    return *m_insertName;
}
