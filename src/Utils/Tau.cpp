#define BIORBD_API_EXPORTS
#include "Utils/Tau.h"

#include "RigidBody/Joints.h"

biorbd::utils::Tau::Tau() {}

biorbd::utils::Tau::Tau(const biorbd::utils::Vector &v) :
    biorbd::utils::Vector(v)
{

}

biorbd::utils::Tau::Tau(unsigned int i) :
    biorbd::utils::Vector(i)
{

}

biorbd::utils::Tau::Tau(const Eigen::VectorXd &v) :
    biorbd::utils::Vector(v)
{

}

biorbd::utils::Tau::Tau(const s2mJoints &j) :
    biorbd::utils::Vector(j.nbTau())
{

}

biorbd::utils::Tau biorbd::utils::Tau::timeDerivativeActivation(const biorbd::utils::Tau &act)
{
    // Implémentation de la fonction da/dt = (u-a)/tau(u,a)
    // ou tau(u,a) = t_act(0.5+1.5*a) is u>a et tau(u,a)=t_deact(0.5+1.5*a) sinon
    //double tauAct = .05;
    double tauDeact = 0.01;

    biorbd::utils::Tau actDot(static_cast<unsigned int>(this->rows()));
//std::cout << "This = " << (*this).transpose() << std::endl;
//std::cout << "act = " << act.transpose() << std::endl;
    for (unsigned int i=0; i<this->rows(); ++i){
        if ((*this)(i) == 0.0)
            actDot(i) = 0;
        else{
            double num = act(i) - (*this)(i) ;
//            std::cout << "num = " << num << std::endl;
            double denom;
//            if (true || num>0 && act(i) > 0 || num<0 && act(i)<0 )
//                denom = tauAct*(0.5+1.5* abs(act(i)));
//            else
                denom = tauDeact/(0.5+1.5* abs(act(i)));
//            std::cout << "denom = "  << denom << std::endl;
            actDot(i) =  num/denom;
//            std::cout << actDot(i) << std::endl;
        }
    }

//    std::cout << "actDot = " << actDot.transpose() << std::endl << std::endl ;
    return actDot;
}
