#define BIORBD_API_EXPORTS
#include "Muscles/MuscleGroup.h"

#include "Utils/Error.h"
#include "Muscles/HillType.h"
#include "Muscles/HillTypeSimple.h"
#include "Muscles/HillTypeThelen.h"
#include "Muscles/HillTypeThelenFatigable.h"
#include "Muscles/StateDynamicsBuchanan.h"

s2mGroupeMusculaire::s2mGroupeMusculaire(
        const biorbd::utils::String &name,
        const biorbd::utils::String &o,
        const biorbd::utils::String &i) :
    m_name(name),
    m_originName(o),
    m_insertName(i)
{
}

s2mGroupeMusculaire::~s2mGroupeMusculaire()
{

}

std::shared_ptr<s2mMuscle> s2mGroupeMusculaire::muscle_nonConst(const unsigned int &idx)
{
    biorbd::utils::Error::error(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return m_mus[idx];
}
const std::shared_ptr<s2mMuscle> s2mGroupeMusculaire::muscle(const unsigned int &idx) const{
    biorbd::utils::Error::error(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return m_mus[idx];
}


void s2mGroupeMusculaire::addHillMuscle(
        const biorbd::utils::String& name,
        const biorbd::utils::String& s,
        const s2mMuscleGeometry& g,
        const s2mMuscleCaracteristics& c,
        const s2mMusclePathChangers& w,
        const biorbd::utils::String& stateType,
        const biorbd::utils::String& dynamicFatigueType){
    s2mMuscleStateDynamics * st;
    if (!stateType.tolower().compare("default"))
        st = new s2mMuscleStateDynamics;
    else if (!stateType.tolower().compare("buchanan"))
        st = new s2mMuscleStateDynamicsBuchanan;
    else {
        st = new s2mMuscleStateDynamics; // remove the warning undeclared
        biorbd::utils::Error::error(false, "Wrong state type");
    }

    if (!s.tolower().compare("hill")){
        s2mMuscleHillType m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillsimple") || !s.tolower().compare("simple")){
        s2mMuscleHillTypeSimple m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillthelen") || !s.tolower().compare("thelen")){
        s2mMuscleHillTypeThelen m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillthelenfatigable") || !s.tolower().compare("thelenfatigable")){
        s2mMuscleHillTypeThelenFatigable m(name,g,c,w,*st,dynamicFatigueType);
        addMuscle(m);
    }
    else
        biorbd::utils::Error::error(0, "Wrong muscle type");


}

void s2mGroupeMusculaire::addMuscle(s2mMuscle &val){

    biorbd::utils::Error::error(muscleID(val.name()) == -1, "This muscle name was already defined for this muscle group");

    // Ajouter un muscle au pool de muscle selon son type
    if (dynamic_cast<s2mMuscleHillTypeSimple*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeSimple(dynamic_cast <s2mMuscleHillTypeSimple&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillTypeThelenFatigable*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeThelenFatigable(dynamic_cast <s2mMuscleHillTypeThelenFatigable&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillTypeThelen*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeThelen(dynamic_cast <s2mMuscleHillTypeThelen&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillType*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillType(dynamic_cast <s2mMuscleHillType&> (val))));
        return;
    }
    else
        biorbd::utils::Error::error(0, "Muscle type not found");

}

unsigned int s2mGroupeMusculaire::nbMuscles() const {
    return static_cast<unsigned int>(m_mus.size());
}


int s2mGroupeMusculaire::muscleID(const biorbd::utils::String& nameToFind){
    std::vector<std::shared_ptr<s2mMuscle>>::iterator musIT=m_mus.begin();
    for (unsigned int i=0; i<m_mus.size(); ++i){
       if (!nameToFind.compare((*(musIT+i))->name()) )
           return static_cast<int>(i);
    }
    // Si on se rend ici, c'est qu'il n'y a pas de muscle de ce nom dans le groupe
    return -1;
}

void s2mGroupeMusculaire::setName(const biorbd::utils::String& name) {
    m_name = name;
}

void s2mGroupeMusculaire::setOrigin(const biorbd::utils::String& name) {
    m_originName = name;
}

void s2mGroupeMusculaire::setInsertion(const biorbd::utils::String& name) {
    m_insertName = name;
}

const biorbd::utils::String &s2mGroupeMusculaire::name() const {
    return m_name;
}

const biorbd::utils::String &s2mGroupeMusculaire::origin() const {
    return m_originName;
}

const biorbd::utils::String &s2mGroupeMusculaire::insertion() const {
    return m_insertName;
}
