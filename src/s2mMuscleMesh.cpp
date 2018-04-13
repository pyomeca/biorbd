#include "../include/s2mMuscleMesh.h"

s2mMuscleMesh::s2mMuscleMesh()
{
    // Nombre de chaque type de muscles
    m_nbMuscles = 0;
    // Declaration of the first part of the table
    m_muscles = new s2mMuscle*[m_nbMuscles];

}

s2mMuscleMesh::~s2mMuscleMesh()
{
    for (unsigned int i=0; i<m_nbMuscles; i++)
        delete m_muscles[i];
    delete[] m_muscles;
}

// Private method to assing values
void s2mMuscleMesh::assignValue(s2mMuscle *&new_m, s2mMuscle *&old_m){
    s2mMuscleHillType   *hill = dynamic_cast<s2mMuscleHillType*>(old_m);
    if (hill){
        new_m = new s2mMuscleHillType;
        (*new_m) = (*old_m);
        return;
    }
    else
        s2mError::s2mAssert(0, "Please update assignValue method in s2mMuscleMesh class!");
}

// Get sur une muscle line
s2mMuscle& s2mMuscleMesh::getMuscleLine(const unsigned int &idx){
    s2mError::s2mAssert(idx<m_nbMuscles, "Index out of bounds for number of muscles");
    return *m_muscles[idx];

}

void s2mMuscleMesh::addLineOfAction(s2mMuscle &val){
    s2mMuscleMesh * tp;
    tp = this; // Copie de la version actuelle en backup

    // Réinitialisation de la variable sphere
    delete[] m_muscles;

    // Incrémenter le nombre de ligne d'action
    m_nbMuscles++;

    m_muscles = new s2mMuscle*[m_nbMuscles];

    // Remettre ce qu'il y avait dans *this avant de l'effacer
    for (unsigned int i=0; i<m_nbMuscles-1; i++)
        assignValue(m_muscles[i], tp->m_muscles[i]);

    // Ajouter la nouvelle ligne
    s2mMuscle * val_tp;
    val_tp = &val;
    assignValue(m_muscles[m_nbMuscles-1], val_tp);
}


s2mMuscleMesh& s2mMuscleMesh::operator= (const s2mMuscleMesh &o){
    // Copier le nombre de wrap par item
    m_nbMuscles = o.m_nbMuscles;

    // Redéclarrer la variable m_wraps et ses composantes
    delete[] m_muscles;
    m_muscles = new s2mMuscle*[m_nbMuscles];

    // Recopier les valeurs
    for (unsigned int i=0; i<m_nbMuscles; i++)
        assignValue(m_muscles[i], o.m_muscles[i]);

    return *this; // Retourner la valeur copiée
}



