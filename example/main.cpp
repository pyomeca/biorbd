#include <iostream>
#include "s2mMusculoSkeletalModel.h"
#include <memory>
#define BUILD_SANDBOX

using namespace std;

void func(){
    s2mGenCoord a(3);

    for (int i = 0; i<3; ++i)
        a[i] = i;
}

#ifdef BUILD_SANDBOX
int main()
{

    s2mMusculoSkeletalModel m3("pyomecaman.bioMod");
    s2mGenCoord Q(m3);
    s2mGenCoord QDDot(m3);
    Q.setZero();
    s2mTau T(m3);
    T.setZero();

    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(m3, Q, Q, T, m3.getConstraints(m3),QDDot);// Forward dynamics
    int i = 0;
    std::cout << m3.getConstraints(m3).force(i) << std::endl;
//    for (int i=0; i<100000000; ++i){
//        std::cout << i << std::endl;
//        RigidBodyDynamics::ForwardDynamicsConstraintsDirect(m3, Q, Q, Q, T);// Inverse Dynamics
//        std::cout << T << std::endl;

//    }
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
