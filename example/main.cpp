#include <iostream>
#include <memory>
#include <cstdlib>
#include <random>
#include "BiorbdModel.h"
#include "RigidBody/GeneralizedCoordinates.h"
#ifdef MODULE_MUSCLES
#include "Muscles/MuscleGroup.h"
#include "Muscles/HillTypeThelenFatigable.h"
#include "Muscles/FatigueDynamicStateXia.h"
#ifdef IPOPT_FOUND
#include "Muscles/StaticOptimization.h"
#endif
#endif // MODULE_MUSCLES
#define BUILD_SANDBOX

#ifdef BUILD_SANDBOX


int main()
{

    biorbd::utils::String path("conv-arm26.bioMod"); // Model path
    biorbd::Model model(path);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedCoordinates QDot(model);
    biorbd::rigidbody::GeneralizedCoordinates QDDot(model);
    for (unsigned i = 0; i<model.nbQ(); i++){
        Q[i] = 0 ;
        QDot[i] = 0;
        QDDot[i] = 0;
    }
#ifdef MODULE_MUSCLES
    model.updateMuscles(Q, QDot, true);

    biorbd::muscles::StateDynamics emg(0, 1);
    // METHOD 1
    {
        std::cout << "Method 1" << std::endl;

        // Showing the initial values
        {
            std::cout << "Initial values" << std::endl;
            biorbd::muscles::HillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Get and modify the values from the model itself
        {
            std::shared_ptr<biorbd::muscles::HillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(model.muscleGroup(0).muscle(0)));
            if (muscle){
                std::shared_ptr<biorbd::muscles::FatigueDynamicStateXia> fatigueModel(std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicStateXia>(muscle->fatigueState()));
                if (fatigueModel){
                    fatigueModel->setState(0.9, 0, 0.1);
                    fatigueModel->timeDerivativeState(emg, model.muscleGroup(0).muscle(0)->caract());
                } else
                    throw std::runtime_error("Fatigue model is not a biorbd::muscles::FatigueDynamicStateXia");
            } else
                throw std::runtime_error("Muscle is not a biorbd::muscles::HillTypeThelenFatigable");
        }

        // Showing that it changed the underlying model values
        {
            std::cout << "Final values" << std::endl;
            biorbd::muscles::HillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }
        std::cout << std::endl;
    }

    // RESET Fatigue param
    {
        std::shared_ptr<biorbd::muscles::FatigueDynamicStateXia> fatigueModel(
                    std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicStateXia>(std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(model.muscleGroup(0).muscle(0))->fatigueState())
                    );
        fatigueModel->setState(1.0, 0, 0);
        fatigueModel->timeDerivativeState(emg, model.muscleGroup(0).muscle(0)->caract());
    }

    // METHOD 2
    {
        std::cout << "Method 2" << std::endl;
        // Showing the initial values
        {
            std::cout << "Initial values" << std::endl;
            biorbd::muscles::HillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Get and modify the values from the model itself
        std::shared_ptr<biorbd::muscles::HillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(model.muscleGroup(0).muscle(0)));
        if (muscle){
            muscle->fatigueState(0.9, 0, 0.1);
            muscle->computeTimeDerivativeState(emg);
        } else
            throw std::runtime_error("Muscle is not a biorbd::muscles::HillTypeThelenFatigable");

        // Showing that it changed the underlying model
        {
            std::cout << "Final values" << std::endl;
            biorbd::muscles::HillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }
        std::cout << std::endl;
    }

    // RESET Fatigue param
    {
        std::shared_ptr<biorbd::muscles::FatigueDynamicStateXia> fatigueModel(
                    std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicStateXia>(std::dynamic_pointer_cast<biorbd::muscles::HillTypeThelenFatigable>(model.muscleGroup(0).muscle(0))->fatigueState())
                    );
        fatigueModel->setState(1.0, 0, 0);
        fatigueModel->timeDerivativeState(emg, model.muscleGroup(0).muscle(0)->caract());
    }

    // METHOD 3 - By copying.. this method don't change the underlying muscle
    {
        std::cout << "Method 3" << std::endl;
        // Showing the initial values
        {
            std::cout << "Initial values" << std::endl;
            biorbd::muscles::HillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Get the values
        {
            std::cout << "Computed values" << std::endl;
            biorbd::muscles::HillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            fatigueModel.setState(0.9, 0, 0.1);
            fatigueModel.timeDerivativeState(emg, model.muscleGroup(0).muscle(0)->caract());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Showing that it doesn't change the underlying model
        {
            std::cout << "Final values" << std::endl;
            biorbd::muscles::HillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            biorbd::muscles::FatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }
        std::cout << std::endl;
    }
#endif // MODULE_MUSCLES

////    biorbd::Model m3("test-os-masse.biomod");
//    biorbd::rigidbody::GeneralizedCoordinates Q(m3);
//    biorbd::rigidbody::GeneralizedCoordinates QDot(m3);
//    biorbd::rigidbody::GeneralizedCoordinates QDDot(m3);

////    std::default_random_engine re(time(0));
////    std::uniform_int_distribution<int> distrib{0, 100};

//    for (unsigned i = 0; i<m3.nbQ(); i++){
//        Q[i] = 0 ;//static_cast<double>(distrib(re)-50)/100*3.14/2;
//        QDot[i] = 0;//static_cast<double>(distrib(re)-50)/100*10;
//        QDDot[i] = 0;//static_cast<double>(distrib(re)-50)/100*50;
//    }
    //QDDot[0] = -11.4546;
    //QDDot[1] = 9.74539;


//    std::vector<biorbd::muscles::StateDynamics> state;
//    for (unsigned int i = 0; i<m3.nbMuscleTotal(); ++i){
//        state.push_back(biorbd::muscles::StateDynamics(0, 0.1));
//    }

//    biorbd::rigidbody::GeneralizedTorque GeneralizedTorque_calcul = m3.muscularJointTorque(m3, state, true, &Q, &QDot);
//    std::cout << "GeneralizedTorque :\n" << GeneralizedTorque_calcul << std::endl;
//    RigidBodyDynamics::ForwardDynamics(m3, Q, QDot, GeneralizedTorque_calcul, QDDot);
//    std::cout << "forward dynamics done" << std::endl;
//    biorbd::rigidbody::GeneralizedTorque GeneralizedTorque_inv(m3.nbGeneralizedTorque());
//    GeneralizedTorque_inv.setZero();
//    RigidBodyDynamics::InverseDynamics(m3, Q, QDot, QDDot, GeneralizedTorque_inv);
//    std::cout << "GeneralizedTorque_inv :\n" << GeneralizedTorque_inv << std::endl;

//    std::vector<biorbd::muscles::StateDynamics> State(m3.nbMuscleTotal());
//    for (unsigned int i = 0; i<m3.nbMuscleTotal(); ++i){
//        State[i].setActivation(0.3925);
//    }
//    //biorbd::rigidbody::GeneralizedTorque GeneralizedTorque_inv = m_model.muscularJointTorque(m_model, m_State, true, &m_Q, &m_Qdot);
//    biorbd::rigidbody::GeneralizedTorque GeneralizedTorque(m3.nbGeneralizedTorque());
//    GeneralizedTorque = m3.muscularJointTorque(m3, State, true, &Q, &QDot);
//    std::cout << "GeneralizedTorque generated by muscle:\n" << GeneralizedTorque << std::endl;

//    unsigned int nbmuscle = m3.nbMuscleTotal();
//    biorbd::utils::Vector activ(nbmuscle);
//    for (unsigned int i=0; i < nbmuscle ; i++){
//        activ[i] = 0.02;
//    }

//    biorbd::utils::Vector a(m3.nbDof());
//    for (unsigned int i=0; i < m3.nbDof() ; i++){
//        a[i] = -9.81;
//    }
//    biorbd::rigidbody::GeneralizedTorque GeneralizedTorque(a);
//    biorbd::muscles::StaticOptimization optim(m3, Q, QDot, GeneralizedTorque, state);
//    optim.optimize();

//    std::cout << "Q:\n" << Q << std::endl;
//    std::cout << "QDot:\n" << QDot << std::endl;
//    std::cout << "QDDot:\n" << QDDot << std::endl;
//    std::cout << "GeneralizedTorque:\n" << GeneralizedTorque << std::endl;



//    biorbd::Model m3("pyomecaman.bioMod");

//    biorbd::rigidbody::GeneralizedCoordinates q;


//    biorbd::rigidbody::GeneralizedCoordinates Q(m3);
//    biorbd::rigidbody::GeneralizedCoordinates QDDot(m3);
//    Q.setZero();
//    biorbd::rigidbody::GeneralizedTorque T(m3);
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

//    biorbd::Model m;
//    m = biorbd::Reader::readModelFile("pyomecaman.bioMod");

//    biorbd::Model * m2 = m3;
//    //m2 = new biorbd::Model(static_cast< const biorbd::Model& >(m));

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
    biorbd::Model m("pyomecaman.bioMod");
    return 0;
}
#endif
