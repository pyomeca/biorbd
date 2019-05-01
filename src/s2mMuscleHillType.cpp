#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleHillType.h"


s2mMuscleHillType::s2mMuscleHillType(const s2mString& name) :
    s2mMuscle(name),
    m_cste_FlCE_1(0.15),
    m_cste_FlCE_2(0.45),
    m_cste_FvCE_1(1),
    m_cste_FvCE_2(-.33/2 * m_cste_FvCE_1/(1+m_cste_FvCE_1)),
    m_cste_FlPE_1(10.0),
    m_cste_FlPE_2(5.0),
    m_cste_forceExcentriqueMultiplier(1.8),
    m_cste_damping(0.1),
    m_cste_vitesseRaccourMax(10.0)

{
    setType();
    setForce();
    //ctor
}
s2mMuscleHillType::s2mMuscleHillType(const s2mMuscleGeometry& g,
                                     const s2mMuscleCaracteristics& c,
                                     const s2mMusclePathChangers & w,
                                     const s2mMuscleStateActual & s) :
    s2mMuscle("",g,c,w,s),
    m_cste_FlCE_1(0.15),
    m_cste_FlCE_2(0.45),
    m_cste_FvCE_1(1),
    m_cste_FvCE_2(-.33/2 * m_cste_FvCE_1/(1+m_cste_FvCE_1)),
    m_cste_FlPE_1(10),
    m_cste_FlPE_2(5),
    m_cste_forceExcentriqueMultiplier(1.8),
    m_cste_damping(0.1),
    m_cste_vitesseRaccourMax(10)
{
    setType();
    setForce();
    //ctor
}
s2mMuscleHillType::s2mMuscleHillType(const s2mString& name,
                                     const s2mMuscleGeometry& g,
                                     const s2mMuscleCaracteristics& c,
                                     const s2mMusclePathChangers & w,
                                     const s2mMuscleStateActual & s) :
    s2mMuscle(name,g,c,w,s),
    m_cste_FlCE_1(0.15),
    m_cste_FlCE_2(0.45),
    m_cste_FvCE_1(1),
    m_cste_FvCE_2(-.33/2 * m_cste_FvCE_1/(1+m_cste_FvCE_1)),
    m_cste_FlPE_1(10),
    m_cste_FlPE_2(5),
    m_cste_forceExcentriqueMultiplier(1.8),
    m_cste_damping(0.1),
    m_cste_vitesseRaccourMax(10)
{
    setType();
    setForce();
    //ctor
}



s2mMuscleHillType::~s2mMuscleHillType()
{
    //dtor
}

void s2mMuscleHillType::setType()
{
    m_type = "Hill";
}

void s2mMuscleHillType::setForce()
{
    m_force.clear();
    std::shared_ptr<s2mMuscleForceFromOrigin> tp_o(new s2mMuscleForceFromOrigin);
    std::shared_ptr<s2mMuscleForceFromInsertion> tp_i(new s2mMuscleForceFromInsertion);
    m_force.push_back(tp_o);
    m_force.push_back(tp_i);
    //dtor
}



std::vector<std::shared_ptr<s2mMuscleForce> > s2mMuscleHillType::force(s2mJoints& m, const s2mGenCoord& Q, const s2mGenCoord& Qdot, const s2mMuscleStateActual& EMG, const int updateKinLevel){
    // Update de la configuration
    if (updateKinLevel == 1)
        updateOrientations(m,Q,Qdot,false);
    else if (updateKinLevel == 2)
        updateOrientations(m,Q,Qdot,true);
    else
        s2mError::s2mAssert(updateKinLevel == 0, "Wrong level of update in force function");

    // Calculs
    return force(EMG);
}

std::vector<std::shared_ptr<s2mMuscleForce> > s2mMuscleHillType::force(const s2mMuscleStateActual &EMG){
    // Calculer chacune les forces dans chaque éléments
    computeFvCE();
    computeFlCE(EMG);
    computeFlPE();
    computeDamping();

    // Combiner les forces
    computeForce(EMG);
    return m_force;
}

s2mMuscleStateActual s2mMuscleHillType::normalizeEMG(s2mMuscleStateActual EMG){
    EMG.excitationNorm(caract().stateMax());
    return EMG;
}

double s2mMuscleHillType::multiplyCaractByActivationAndForce(const s2mMuscleStateActual &EMG){
    // Fonction qui permet de modifier la facon dont la multiplication est faite dans computeForce(EMG)
    return caract().forceIsoMax() * (EMG.activation()*m_FlCE*m_FvCE + m_FlPE + m_damping);
}

void s2mMuscleHillType::computeForce(const s2mMuscleStateActual &EMG){
    double force = multiplyCaractByActivationAndForce(EMG);
    m_force[0]->setForce(m_position, force); // origine vers le deuxieme point
    m_force[1]->setForce(m_position, force); // insertion vers l'avant-dernier point
}

void s2mMuscleHillType::computeFlPE(){
	if (m_position.length() > caract().tendonSlackLength())
		m_FlPE = exp(m_cste_FlPE_1*(m_position.length()/caract().optimalLength()-1) - m_cste_FlPE_2);
	else 
		m_FlPE = 0;
    }

void s2mMuscleHillType::computeDamping(){
    m_damping = m_position.velocity() / (m_cste_vitesseRaccourMax * caract().optimalLength()) * m_cste_damping;
}

void s2mMuscleHillType::computeFlCE(const s2mMuscleStateActual &EMG){
    m_FlCE = exp( -pow(( m_position.length() / caract().optimalLength() / (m_cste_FlCE_1*(1-EMG.activation())+1) -1 ), 2)/m_cste_FlCE_2   );
}

void s2mMuscleHillType::computeFvCE(){
    // La relation est différente si la vitesse < 0  ou > 0
    double v = m_position.velocity();
    if (v<=0)
        m_FvCE = (1-abs(v)/m_cste_vitesseRaccourMax) /
            (1+abs(v)/m_cste_vitesseRaccourMax/m_cste_FvCE_1);
    else
        m_FvCE = (1-1.33*v/m_cste_vitesseRaccourMax/m_cste_FvCE_2) /
                   (1-v/m_cste_vitesseRaccourMax/m_cste_FvCE_2);
}






