#define BIORBD_API_EXPORTS
#include "s2mGroupeMusculaire.h"

#include "Utils/Error.h"
#include "s2mMuscleHillType.h"
#include "s2mMuscleHillTypeSimple.h"
#include "s2mMuscleHillTypeThelen.h"
#include "s2mMuscleHillTypeThelenFatigable.h"
#include "s2mMuscleStateDynamicsBuchanan.h"

s2mGroupeMusculaire::s2mGroupeMusculaire(const s2mString &name, const s2mString &o, const s2mString &i) :
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
    s2mError::s2mAssert(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return m_mus[idx];
}
const std::shared_ptr<s2mMuscle> s2mGroupeMusculaire::muscle(const unsigned int &idx) const{
    s2mError::s2mAssert(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return m_mus[idx];
}


void s2mGroupeMusculaire::addHillMuscle(
        const s2mString& name,
        const s2mString& s,
        const s2mMuscleGeometry& g,
        const s2mMuscleCaracteristics& c,
        const s2mMusclePathChangers& w,
        const s2mString& stateType,
        const s2mString& dynamicFatigueType){
    s2mMuscleStateDynamics * st;
    if (!stateType.tolower().compare("default"))
        st = new s2mMuscleStateDynamics;
    else if (!stateType.tolower().compare("buchanan"))
        st = new s2mMuscleStateDynamicsBuchanan;
    else {
        st = new s2mMuscleStateDynamics; // remove the warning undeclared
        s2mError::s2mAssert(false, "Wrong state type");
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
        s2mError::s2mAssert(0, "Wrong muscle type");


}

void s2mGroupeMusculaire::addMuscle(s2mMuscle &val){

    s2mError::s2mAssert(muscleID(val.name()) == -1, "This muscle name was already defined for this muscle group");

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
        s2mError::s2mAssert(0, "Muscle type not found");

}

unsigned int s2mGroupeMusculaire::nbMuscles() const {
    return static_cast<unsigned int>(m_mus.size());
}


int s2mGroupeMusculaire::muscleID(const s2mString& nameToFind){
    std::vector<std::shared_ptr<s2mMuscle>>::iterator musIT=m_mus.begin();
    for (unsigned int i=0; i<m_mus.size(); ++i){
       if (!nameToFind.compare((*(musIT+i))->name()) )
           return static_cast<int>(i);
    }
    // Si on se rend ici, c'est qu'il n'y a pas de muscle de ce nom dans le groupe
    return -1;
}

void s2mGroupeMusculaire::setName(s2mString name) {
    m_name = name;
}

void s2mGroupeMusculaire::setOrigin(s2mString name) {
    m_originName = name;
}

void s2mGroupeMusculaire::setInsertion(s2mString name) {
    m_insertName = name;
}

const s2mString &s2mGroupeMusculaire::name() const {
    return m_name;
}

const s2mString &s2mGroupeMusculaire::origin() const {
    return m_originName;
}

const s2mString &s2mGroupeMusculaire::insertion() const {
    return m_insertName;
}
