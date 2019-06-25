#include <iostream>
#include "s2mMusculoSkeletalModel.h"
#include "s2mStaticOptimization.h"
#include <memory>
#include "IpoptTestMain.h"
#define BUILD_SANDBOX

using namespace std;

void func(){
    s2mGenCoord a(3);

    for (int i = 0; i<3; ++i){
        a[i] = i;
    }
}

#ifdef BUILD_SANDBOX


int main()
{

    //mainTest run;
    //run.main();

    s2mMusculoSkeletalModel m3("Bras.bioMod");
    s2mGenCoord q;
    s2mGenCoord Q(m3);
    s2mGenCoord QDDot(m3);
    s2mGenCoord QDot(m3);
    Q.setZero();
    QDot.setZero();
    s2mTau T(m3);
    T.setZero();

    for (unsigned int i=0; i < m3.nbTau() ; i++){
        T[i] = 2;
    }


    unsigned int nbmuscle = m3.nbMuscleTotal();
    s2mVector activ(nbmuscle);
    for (unsigned int i=0; i < nbmuscle ; i++){
        activ[i] = 0.02;
    }


    std::vector<s2mMuscleStateActual> state;// controls
    for (unsigned int i = 0; i<nbmuscle; ++i){
        std::cout << "m_activation[" << i << "]: " << activ[i] << std::endl;
        state.push_back(s2mMuscleStateActual(0, activ[i]));
        std::cout << "state.push" << std::endl;
    }

    // Compute the torques from muscles
    m3.updateMuscles(m3, Q, QDot, true);
    s2mTau tau_calcul = m3.muscularJointTorque(m3, state, true, &Q, &QDot);
    std::cout << "tau_calcul" << tau_calcul << std::endl;


    s2mStaticOptimization optim(m3, Q, QDot, T, activ);

    optim.run(m3, Q, QDot, T, activ, 2);


//    s2mMusculoSkeletalModel m3("pyomecaman.bioMod");

//    s2mGenCoord q;


//    s2mGenCoord Q(m3);
//    s2mGenCoord QDDot(m3);
//    Q.setZero();
//    s2mTau T(m3);
//    T.setZero();

//    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(m3, Q, Q, T, m3.getConstraints(m3),QDDot);// Forward dynamics
//    int i = 0;
//    std::cout << m3.getConstraints(m3).force(i) << std::endl;
////    for (int i=0; i<100000000; ++i){
////        std::cout << i << std::endl;
////        RigidBodyDynamics::ForwardDynamicsConstraintsDirect(m3, Q, Q, Q, T);// Inverse Dynamics
////        std::cout << T << std::endl;

////    }
    return 0;

//    s2mMusculoSkeletalModel m;
//    m = s2mRead::readModelFile("pyomecaman.s2mMod");

//    s2mMusculoSkeletalModel * m2 = m3;
//    //m2 = new s2mMusculoSkeletalModel(static_cast< const s2mMusculoSkeletalModel& >(m));

//    std::cout << m2->nbQ() << std::endl;
//    delete m2;
//    delete m3;
//    std::cout << m3->nbQ() << std::endl;
//    std::cout << m2->nbQ() << std::endl;


//    std::cout << m.Tags((unsigned int)0) << std::endl;
//    std::cout << m2->Tags((unsigned int)0) << std::endl;
//    std::cout << m3->Tags((unsigned int)0) << std::endl;
//    //for (int i = 0; i<10000; ++i)
//    //    func();
//    delete m2;
//    return 0;
}

#else
int main()
{
    s2mMusculoSkeletalModel m("pyomecaman.s2mMod");
    return 0;
}
#endif
