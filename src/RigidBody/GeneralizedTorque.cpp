#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedTorque.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque() {}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(const biorbd::rigidbody::GeneralizedTorque &Q) :
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(unsigned int i) :
    biorbd::utils::Vector(i)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbGeneralizedTorque())
{

}

biorbd::rigidbody::GeneralizedTorque biorbd::rigidbody::GeneralizedTorque::DeepCopy() const
{
    biorbd::rigidbody::GeneralizedTorque copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::GeneralizedTorque::DeepCopy(const biorbd::rigidbody::GeneralizedTorque &other)
{
    biorbd::utils::Vector::DeepCopy(other);
}

biorbd::rigidbody::GeneralizedTorque biorbd::rigidbody::GeneralizedTorque::timeDerivativeActivation(const biorbd::rigidbody::GeneralizedTorque &act)
{
    // Implémentation de la fonction da/dt = (u-a)/GeneralizedTorque(u,a)
    // ou GeneralizedTorque(u,a) = t_act(0.5+1.5*a) is u>a et GeneralizedTorque(u,a)=t_deact(0.5+1.5*a) sinon
    //double GeneralizedTorqueAct = .05;
    double GeneralizedTorqueDeact = 0.01;

    biorbd::rigidbody::GeneralizedTorque actDot(static_cast<unsigned int>(this->rows()));
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
//                denom = GeneralizedTorqueAct*(0.5+1.5* abs(act(i)));
//            else
                denom = GeneralizedTorqueDeact/(0.5+1.5* abs(act(i)));
//            std::cout << "denom = "  << denom << std::endl;
            actDot(i) =  num/denom;
//            std::cout << actDot(i) << std::endl;
        }
    }

//    std::cout << "actDot = " << actDot.transpose() << std::endl << std::endl ;
    return actDot;
}
