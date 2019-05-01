#define BIORBD_API_EXPORTS
#include "../include/s2mActuators.h"


s2mActuators::s2mActuators() :
    m_isClose(false)
{
    m_isDofSet = new bool[1];
    m_isDofSet[0] = false;
}

s2mActuators::s2mActuators(const s2mActuators& a) :
    m_isClose(false)
{
    m_isClose = a.m_isClose;
    m_isDofSet = new bool[1];
    m_isDofSet[0] = a.m_isDofSet[0];
    m_all = a.m_all;
}


s2mActuators::~s2mActuators(){
    delete[] m_isDofSet;
}



void s2mActuators::addActuator(const s2mJoints& m, s2mActuator &act){
    s2mError::s2mAssert(!m_isClose, "You can't add actuator after closing the model");

    // Vérifier que le dof target est associé à un dof qui existe déjà dans le modèle
    s2mError::s2mAssert(act.index()<m.nbDof(), "Sent index is out of dof range");

    // Pour fin de rapidité et de cohérence avec les Q, mettre l'actuator au même index que son dof associé
    unsigned int idx(act.index());

    // S'il y a moins d'actuateurs déjà déclaré qu'il n'y a de dof, il faut agrandir le vecteur
    if (idx >= m_all.size()){
        unsigned int oldSize = m_all.size();
        bool * isDofSet = new bool[oldSize*2+1];
        for (unsigned int i=0; i<oldSize*2; ++i)
            isDofSet[i] = m_isDofSet[i];
        m_all.resize(idx+1);

        delete[] m_isDofSet;
        m_isDofSet = new bool[idx*2+1];
        for (unsigned int i=0; i<m_all.size()*2; ++i)
            if (i<oldSize*2)
                m_isDofSet[i] = isDofSet[i];
            else
                m_isDofSet[i] = false;

    }

    // Ajouter un actuator au pool d'actuator selon son type
    if (dynamic_cast<s2mActuatorGauss3p*> (&act)){
        if (act.direction() == 1){
            m_all[idx].first = std::shared_ptr<s2mActuator> (new s2mActuatorGauss3p(dynamic_cast <s2mActuatorGauss3p&> (act)));
            m_isDofSet[idx*2] = true;
        }
        else{
            m_all[idx].second = std::shared_ptr<s2mActuator> (new s2mActuatorGauss3p(dynamic_cast <s2mActuatorGauss3p&> (act)));
            m_isDofSet[idx*2+1] = true;
        }
        return;
    }
    else if (dynamic_cast<s2mActuatorConstant*> (&act)){
        if (act.direction() == 1){
            m_all[idx].first = std::shared_ptr<s2mActuator> (new s2mActuatorConstant(dynamic_cast <s2mActuatorConstant&> (act)));
            m_isDofSet[idx*2] = true;
        }
        else{
            m_all[idx].second = std::shared_ptr<s2mActuator> (new s2mActuatorConstant(dynamic_cast <s2mActuatorConstant&> (act)));
            m_isDofSet[idx*2+1] = true;
        }
        return;
    }
    else if (dynamic_cast<s2mActuatorLinear*> (&act)){
        if (act.direction() == 1){
            m_all[idx].first = std::shared_ptr<s2mActuator> (new s2mActuatorLinear(dynamic_cast <s2mActuatorLinear&> (act)));
            m_isDofSet[idx*2] = true;
        }
        else{
            m_all[idx].second = std::shared_ptr<s2mActuator> (new s2mActuatorLinear(dynamic_cast <s2mActuatorLinear&> (act)));
            m_isDofSet[idx*2+1] = true;
        }
        return;
    }
    else if (dynamic_cast<s2mActuatorGauss6p*> (&act)){
        if (act.direction() == 1){
            m_all[idx].first = std::shared_ptr<s2mActuator> (new s2mActuatorGauss6p(dynamic_cast <s2mActuatorGauss6p&> (act)));
            m_isDofSet[idx*2] = true;
        }
        else{
            m_all[idx].second = std::shared_ptr<s2mActuator> (new s2mActuatorGauss6p(dynamic_cast <s2mActuatorGauss6p&> (act)));
            m_isDofSet[idx*2+1] = true;
        }
        return;
    }
    else
        s2mError::s2mAssert(0, "Actuator type not found");

}

void s2mActuators::closeActuator(s2mJoints& m){
    s2mError::s2mAssert(m.nbDof()==m_all.size(), "All dof must have their actuators set");

    for (unsigned int i=0; i<m_all.size()*2; ++i)
        s2mError::s2mAssert(m_isDofSet[i], "All DoF must have their actuators set before closing the model");

    m_isClose = true;
}

std::pair<std::shared_ptr<s2mActuator>, std::shared_ptr<s2mActuator> > s2mActuators::actuator(unsigned int dof){
    s2mError::s2mAssert(dof<nbActuators(), "Idx asked is higher than number of actuator");
    return *(m_all.begin() + dof);
}
std::shared_ptr<s2mActuator> s2mActuators::actuator(unsigned int dof, unsigned int idx){
    std::pair<std::shared_ptr<s2mActuator>, std::shared_ptr<s2mActuator> > tp(actuator(dof));

    s2mError::s2mAssert(idx == 0 || idx == 1, "Index of actuator should be 0 or 1");
    if (idx == 0)
        return tp.first;
    else
        return tp.second;
}

s2mTau s2mActuators::torque(const s2mJoints& m, const s2mGenCoord& a, const s2mGenCoord& Q, const s2mGenCoord &Qdot){

    // Mettre pour que qdot soit positif en concentrique et negatif en excentrique
    s2mGenCoord QdotResigned(Qdot);
    for (unsigned int i=0; i<Qdot.size(); ++i)
        if (a(i)<0)
            QdotResigned(i) = -Qdot(i);

    // Calcul des torque sous maximaux
    s2mTau Tau(torqueMax(m, a,Q,QdotResigned));

    // Remettre les signes
    for (unsigned int i=0; i<Tau.size(); ++i)
        Tau(i) *= a(i);

    return Tau;
}


std::pair<s2mTau, s2mTau> s2mActuators::torqueMax(const s2mJoints &m, const s2mGenCoord& Q, const s2mGenCoord &Qdot){
    s2mError::s2mAssert(m_isClose, "Close the actuator model before calling torqueMax");

    std::pair<s2mTau, s2mTau> maxTau_all = std::make_pair(s2mTau(m), s2mTau(m));

    for (unsigned int i=0; i<m.nbDof(); ++i){
        std::pair<std::shared_ptr<s2mActuator>, std::shared_ptr<s2mActuator> > Tau_tp(actuator(i));
        for (unsigned p=0; p<2; ++p){
            if (p==0) // First
                if (std::dynamic_pointer_cast<s2mActuatorGauss3p> (Tau_tp.first))
                   maxTau_all.first[i] = std::static_pointer_cast<s2mActuatorGauss3p> (Tau_tp.first)->torqueMax(Q, Qdot);
                else if (std::dynamic_pointer_cast<s2mActuatorConstant> (Tau_tp.first))
                    maxTau_all.first[i] = std::static_pointer_cast<s2mActuatorConstant> (Tau_tp.first)->torqueMax();
                else if (std::dynamic_pointer_cast<s2mActuatorLinear> (Tau_tp.first))
                    maxTau_all.first[i] = std::static_pointer_cast<s2mActuatorLinear> (Tau_tp.first)->torqueMax(Q);
                else if (std::dynamic_pointer_cast<s2mActuatorGauss6p> (Tau_tp.first))
                    maxTau_all.first[i] = std::static_pointer_cast<s2mActuatorGauss6p> (Tau_tp.first)->torqueMax(Q, Qdot);
                else
                    s2mError::s2mAssert(false, "Wrong type (should never get here because of previous safety)");
            else // Second
                if (std::dynamic_pointer_cast<s2mActuatorGauss3p> (Tau_tp.second))
                    maxTau_all.second[i] = std::static_pointer_cast<s2mActuatorGauss3p> (Tau_tp.second)->torqueMax(Q, Qdot);
                else if (std::dynamic_pointer_cast<s2mActuatorConstant> (Tau_tp.second))
                    maxTau_all.second[i] = std::static_pointer_cast<s2mActuatorConstant> (Tau_tp.second)->torqueMax();
                else if (std::dynamic_pointer_cast<s2mActuatorLinear> (Tau_tp.second))
                    maxTau_all.second[i] = std::static_pointer_cast<s2mActuatorLinear> (Tau_tp.second)->torqueMax(Q);
                else if (std::dynamic_pointer_cast<s2mActuatorGauss6p> (Tau_tp.second))
                    maxTau_all.second[i] = std::static_pointer_cast<s2mActuatorGauss6p> (Tau_tp.second)->torqueMax(Q, Qdot);
                else
                    s2mError::s2mAssert(false, "Wrong type (should never get here because of previous safety)");
        }
    }

    return maxTau_all;
}


s2mTau s2mActuators::torqueMax(const s2mJoints &m, const s2mGenCoord& a, const s2mGenCoord& Q, const s2mGenCoord &Qdot){
    s2mError::s2mAssert(m_isClose, "Close the actuator model before calling torqueMax");

    s2mTau maxTau_all;
    maxTau_all.resize(m.nbDof());

    for (unsigned int i=0; i<m.nbDof(); ++i){
        std::shared_ptr<s2mActuator> Tau_tp;
        if (a[i]>=0) // First
            Tau_tp = actuator(i).first;
        else
            Tau_tp = actuator(i).second;

        if (std::dynamic_pointer_cast<s2mActuatorGauss3p> (Tau_tp))
            maxTau_all[i] = std::static_pointer_cast<s2mActuatorGauss3p> (Tau_tp)->torqueMax(Q, Qdot);
        else if (std::dynamic_pointer_cast<s2mActuatorConstant> (Tau_tp))
            maxTau_all[i] = std::static_pointer_cast<s2mActuatorConstant> (Tau_tp)->torqueMax();
        else if (std::dynamic_pointer_cast<s2mActuatorLinear> (Tau_tp))
            maxTau_all[i] = std::static_pointer_cast<s2mActuatorLinear> (Tau_tp)->torqueMax(Q);
        else if (std::dynamic_pointer_cast<s2mActuatorGauss6p> (Tau_tp))
            maxTau_all[i] = std::static_pointer_cast<s2mActuatorGauss6p> (Tau_tp)->torqueMax(Q, Qdot);
        else
            s2mError::s2mAssert(false, "Wrong type (should never get here because of previous safety)");

    }

    return maxTau_all;
}
