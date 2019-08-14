#define BIORBD_API_EXPORTS
#include "Muscles/MuscleGroup.h"

#include "Utils/Error.h"
#include "Muscles/HillType.h"
#include "Muscles/HillTypeSimple.h"
#include "Muscles/HillTypeThelen.h"
#include "Muscles/HillTypeThelenFatigable.h"
#include "Muscles/StateDynamicsBuchanan.h"

biorbd::muscles::MuscleGroup::MuscleGroup(
        const biorbd::utils::String &name,
        const biorbd::utils::String &o,
        const biorbd::utils::String &i) :
    m_name(name),
    m_originName(o),
    m_insertName(i)
{
}

biorbd::muscles::MuscleGroup::~MuscleGroup()
{

}

std::shared_ptr<biorbd::muscles::Muscle> biorbd::muscles::MuscleGroup::muscle_nonConst(const unsigned int &idx)
{
    biorbd::utils::Error::error(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return m_mus[idx];
}
const std::shared_ptr<biorbd::muscles::Muscle> biorbd::muscles::MuscleGroup::muscle(const unsigned int &idx) const{
    biorbd::utils::Error::error(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return m_mus[idx];
}


void biorbd::muscles::MuscleGroup::addHillMuscle(
        const biorbd::utils::String& name,
        const biorbd::utils::String& s,
        const biorbd::muscles::Geometry& g,
        const biorbd::muscles::Caracteristics& c,
        const biorbd::muscles::PathChangers& w,
        const biorbd::utils::String& stateType,
        const biorbd::utils::String& dynamicFatigueType){
    biorbd::muscles::StateDynamics * st;
    if (!stateType.tolower().compare("default"))
        st = new biorbd::muscles::StateDynamics;
    else if (!stateType.tolower().compare("buchanan"))
        st = new biorbd::muscles::StateDynamicsBuchanan;
    else {
        st = new biorbd::muscles::StateDynamics; // remove the warning undeclared
        biorbd::utils::Error::error(false, "Wrong state type");
    }

    if (!s.tolower().compare("hill")){
        biorbd::muscles::HillType m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillsimple") || !s.tolower().compare("simple")){
        biorbd::muscles::HillTypeSimple m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillthelen") || !s.tolower().compare("thelen")){
        biorbd::muscles::HillTypeThelen m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillthelenfatigable") || !s.tolower().compare("thelenfatigable")){
        biorbd::muscles::HillTypeThelenFatigable m(name,g,c,w,*st,dynamicFatigueType);
        addMuscle(m);
    }
    else
        biorbd::utils::Error::error(0, "Wrong muscle type");


}

void biorbd::muscles::MuscleGroup::addMuscle(biorbd::muscles::Muscle &val){

    biorbd::utils::Error::error(muscleID(val.name()) == -1, "This muscle name was already defined for this muscle group");

    // Ajouter un muscle au pool de muscle selon son type
    if (dynamic_cast<biorbd::muscles::HillTypeSimple*> (&val)){
        m_mus.push_back(std::shared_ptr<biorbd::muscles::Muscle> (new biorbd::muscles::HillTypeSimple(dynamic_cast <biorbd::muscles::HillTypeSimple&> (val))));
        return;
    }
    else if (dynamic_cast<biorbd::muscles::HillTypeThelenFatigable*> (&val)){
        m_mus.push_back(std::shared_ptr<biorbd::muscles::Muscle> (new biorbd::muscles::HillTypeThelenFatigable(dynamic_cast <biorbd::muscles::HillTypeThelenFatigable&> (val))));
        return;
    }
    else if (dynamic_cast<biorbd::muscles::HillTypeThelen*> (&val)){
        m_mus.push_back(std::shared_ptr<biorbd::muscles::Muscle> (new biorbd::muscles::HillTypeThelen(dynamic_cast <biorbd::muscles::HillTypeThelen&> (val))));
        return;
    }
    else if (dynamic_cast<biorbd::muscles::HillType*> (&val)){
        m_mus.push_back(std::shared_ptr<biorbd::muscles::Muscle> (new biorbd::muscles::HillType(dynamic_cast <biorbd::muscles::HillType&> (val))));
        return;
    }
    else
        biorbd::utils::Error::error(0, "Muscle type not found");

}

unsigned int biorbd::muscles::MuscleGroup::nbMuscles() const {
    return static_cast<unsigned int>(m_mus.size());
}


int biorbd::muscles::MuscleGroup::muscleID(const biorbd::utils::String& nameToFind){
    std::vector<std::shared_ptr<biorbd::muscles::Muscle>>::iterator musIT=m_mus.begin();
    for (unsigned int i=0; i<m_mus.size(); ++i){
       if (!nameToFind.compare((*(musIT+i))->name()) )
           return static_cast<int>(i);
    }
    // Si on se rend ici, c'est qu'il n'y a pas de muscle de ce nom dans le groupe
    return -1;
}

void biorbd::muscles::MuscleGroup::setName(const biorbd::utils::String& name) {
    m_name = name;
}

void biorbd::muscles::MuscleGroup::setOrigin(const biorbd::utils::String& name) {
    m_originName = name;
}

void biorbd::muscles::MuscleGroup::setInsertion(const biorbd::utils::String& name) {
    m_insertName = name;
}

const biorbd::utils::String &biorbd::muscles::MuscleGroup::name() const {
    return m_name;
}

const biorbd::utils::String &biorbd::muscles::MuscleGroup::origin() const {
    return m_originName;
}

const biorbd::utils::String &biorbd::muscles::MuscleGroup::insertion() const {
    return m_insertName;
}
