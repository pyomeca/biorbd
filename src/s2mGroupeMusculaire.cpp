#define BIORBD_API_EXPORTS
#include "../include/s2mGroupeMusculaire.h"

s2mGroupeMusculaire::s2mGroupeMusculaire(const s2mString &name, const s2mString &o, const s2mString &i) :
    m_name(name),
    m_originName(o),
    m_insertName(i)
{
}

s2mGroupeMusculaire::~s2mGroupeMusculaire()
{
}

std::shared_ptr<s2mMuscle> s2mGroupeMusculaire::muscle(const unsigned int &idx){
    s2mError::s2mAssert(idx<nbMuscles(), "Idx asked is higher than number of muscles");
    return *(m_mus.begin() + idx);
}


void s2mGroupeMusculaire::addHillMuscle(const s2mString& name, const s2mString& s, const s2mMuscleGeometry& g, const s2mMuscleCaracteristics& c, const s2mMusclePathChangers& w, const s2mString& stateType){
    s2mMuscleStateActual * st;
    if (!stateType.tolower().compare("default"))
        st = new s2mMuscleStateActual;
    else if (!stateType.tolower().compare("buchanan"))
        st = new s2mMuscleStateActualBuchanan;
    else {
        st = new s2mMuscleStateActual; // remove the warning undeclared
        s2mError::s2mAssert(false, "Wrong state type");
    }

    if (!s.tolower().compare("hill")){
        s2mMuscleHillType m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillmax")){
        s2mMuscleHillTypeMaxime m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillchadwick")){
        s2mMuscleHillTypeChadwick m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillthelen") || !s.tolower().compare("thelen")){
        s2mMuscleHillTypeThelen m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillschutte") || !s.tolower().compare("schutte")){
        s2mMuscleHillTypeSchutte m(name,g,c,w,*st);
        addMuscle(m);
    }
    else if (!s.tolower().compare("hillsimple") || !s.tolower().compare("simple")){
        s2mMuscleHillTypeSimple m(name,g,c,w,*st);
        addMuscle(m);
    }
    else
        s2mError::s2mAssert(0, "Wrong muscle type");


}

void s2mGroupeMusculaire::addMuscle(s2mMuscle &val){

    s2mError::s2mAssert(muscleID(val.name()) == -1, "This muscle name was already defined for this muscle group");

    // Ajouter un muscle au pool de muscle selon son type
    if (dynamic_cast<s2mMuscleHillTypeMaxime*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeMaxime(dynamic_cast <s2mMuscleHillTypeMaxime&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillTypeSimple*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeSimple(dynamic_cast <s2mMuscleHillTypeSimple&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillTypeChadwick*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeChadwick(dynamic_cast <s2mMuscleHillTypeChadwick&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillTypeThelen*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeThelen(dynamic_cast <s2mMuscleHillTypeThelen&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillTypeSchutte*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillTypeSchutte(dynamic_cast <s2mMuscleHillTypeSchutte&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleMeshTransverse*> (&val)){
        //m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleMeshTransverse(dynamic_cast <s2mMuscleMeshTransverse&> (val))));
        return;
    }
    else if (dynamic_cast<s2mMuscleHillType*> (&val)){
        m_mus.push_back(std::shared_ptr<s2mMuscle> (new s2mMuscleHillType(dynamic_cast <s2mMuscleHillType&> (val))));
        return;
    }
    else
        s2mError::s2mAssert(0, "Muscle type not found");

}

unsigned int s2mGroupeMusculaire::nbMuscles() const { return m_mus.size(); }


int s2mGroupeMusculaire::muscleID(const s2mString& nameToFind){
    std::vector<std::shared_ptr<s2mMuscle> >::iterator musIT=m_mus.begin();
    for (unsigned int i=0; i<m_mus.size(); ++i){
       if (!nameToFind.compare((*(musIT+i))->name()) )
           return i;
    }
    // Si on se rend ici, c'est qu'il n'y a pas de muscle de ce nom dans le groupe
    return -1;
}

void s2mGroupeMusculaire::setName(s2mString name) {m_name = name;}

void s2mGroupeMusculaire::setOrigin(s2mString name) {m_originName = name;}

void s2mGroupeMusculaire::setInsertion(s2mString name) {m_insertName = name;}

s2mString s2mGroupeMusculaire::name() const { return m_name;}

s2mString s2mGroupeMusculaire::origin() const {return m_originName;}

s2mString s2mGroupeMusculaire::insertion() const {return m_insertName;}
