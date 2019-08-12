#include <iostream>
#include "s2mMusculoSkeletalModel.h"
#include "s2mStaticOptimization.h"
#include "Utils/GenCoord.h"
#include "s2mGroupeMusculaire.h"
#include "s2mMuscleHillTypeThelenFatigable.h"
#include "s2mMuscleFatigueDynamicStateXia.h"
#include <memory>
#include <cstdlib>
#include <random>
#define BUILD_SANDBOX

#ifdef BUILD_SANDBOX


int main()
{

    s2mString path("conv-arm26.bioMod"); // Model path
    s2mMusculoSkeletalModel model(path);
    biorbd::utils::GenCoord Q(model);
    biorbd::utils::GenCoord QDot(model);
    biorbd::utils::GenCoord QDDot(model);
    for (unsigned i = 0; i<model.nbQ(); i++){
        Q[i] = 0 ;
        QDot[i] = 0;
        QDDot[i] = 0;
    }
    model.updateMuscles(model, Q, QDot, true);

    s2mMuscleStateDynamics emg(0, 1);
    // METHOD 1
    {
        std::cout << "Method 1" << std::endl;

        // Showing the initial values
        {
            std::cout << "Initial values" << std::endl;
            s2mMuscleHillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Get and modify the values from the model itself
        {
            std::shared_ptr<s2mMuscleHillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(model.muscleGroup(0).muscle(0)));
            if (muscle){
                std::shared_ptr<s2mMuscleFatigueDynamicStateXia> fatigueModel(std::dynamic_pointer_cast<s2mMuscleFatigueDynamicStateXia>(muscle->fatigueState()));
                if (fatigueModel){
                    fatigueModel->setState(0.9, 0, 0.1);
                    fatigueModel->timeDerivativeState(emg, model.muscleGroup(0).muscle(0)->caract());
                } else
                    throw std::runtime_error("Fatigue model is not a s2mMuscleFatigueDynamicStateXia");
            } else
                throw std::runtime_error("Muscle is not a s2mMuscleHillTypeThelenFatigable");
        }

        // Showing that it changed the underlying model values
        {
            std::cout << "Final values" << std::endl;
            s2mMuscleHillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }
        std::cout << std::endl;
    }

    // RESET Fatigue param
    {
        std::shared_ptr<s2mMuscleFatigueDynamicStateXia> fatigueModel(
                    std::dynamic_pointer_cast<s2mMuscleFatigueDynamicStateXia>(std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(model.muscleGroup(0).muscle(0))->fatigueState())
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
            s2mMuscleHillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Get and modify the values from the model itself
        std::shared_ptr<s2mMuscleHillTypeThelenFatigable> muscle(std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(model.muscleGroup(0).muscle(0)));
        if (muscle){
            muscle->fatigueState(0.9, 0, 0.1);
            muscle->computeTimeDerivativeState(emg);
        } else
            throw std::runtime_error("Muscle is not a s2mMuscleHillTypeThelenFatigable");

        // Showing that it changed the underlying model
        {
            std::cout << "Final values" << std::endl;
            s2mMuscleHillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }
        std::cout << std::endl;
    }

    // RESET Fatigue param
    {
        std::shared_ptr<s2mMuscleFatigueDynamicStateXia> fatigueModel(
                    std::dynamic_pointer_cast<s2mMuscleFatigueDynamicStateXia>(std::dynamic_pointer_cast<s2mMuscleHillTypeThelenFatigable>(model.muscleGroup(0).muscle(0))->fatigueState())
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
            s2mMuscleHillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Get the values
        {
            std::cout << "Computed values" << std::endl;
            s2mMuscleHillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            fatigueModel.setState(0.9, 0, 0.1);
            fatigueModel.timeDerivativeState(emg, model.muscleGroup(0).muscle(0)->caract());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }

        // Showing that it doesn't change the underlying model
        {
            std::cout << "Final values" << std::endl;
            s2mMuscleHillTypeThelenFatigable muscle(model.muscleGroup(0).muscle(0));
            s2mMuscleFatigueDynamicStateXia fatigueModel(muscle.fatigueState());
            std::cout << "activeFibersDot = " << fatigueModel.activeFibersDot() << std::endl;
            std::cout << "fatiguedFibersDot = " << fatigueModel.fatiguedFibersDot() << std::endl;
            std::cout << "restingFibersDot = " << fatigueModel.restingFibersDot() << std::endl;
        }
        std::cout << std::endl;
    }


////    s2mMusculoSkeletalModel m3("test-os-masse.biomod");
//    biorbd::utils::GenCoord Q(m3);
//    biorbd::utils::GenCoord QDot(m3);
//    biorbd::utils::GenCoord QDDot(m3);

////    std::default_random_engine re(time(0));
////    std::uniform_int_distribution<int> distrib{0, 100};

//    for (unsigned i = 0; i<m3.nbQ(); i++){
//        Q[i] = 0 ;//static_cast<double>(distrib(re)-50)/100*3.14/2;
//        QDot[i] = 0;//static_cast<double>(distrib(re)-50)/100*10;
//        QDDot[i] = 0;//static_cast<double>(distrib(re)-50)/100*50;
//    }
    //QDDot[0] = -11.4546;
    //QDDot[1] = 9.74539;


//    std::vector<s2mMuscleStateDynamics> state;
//    for (unsigned int i = 0; i<m3.nbMuscleTotal(); ++i){
//        state.push_back(s2mMuscleStateDynamics(0, 0.1));
//    }

//    s2mTau tau_calcul = m3.muscularJointTorque(m3, state, true, &Q, &QDot);
//    std::cout << "tau :\n" << tau_calcul << std::endl;
//    RigidBodyDynamics::ForwardDynamics(m3, Q, QDot, tau_calcul, QDDot);
//    std::cout << "forward dynamics done" << std::endl;
//    s2mTau tau_inv(m3.nbTau());
//    tau_inv.setZero();
//    RigidBodyDynamics::InverseDynamics(m3, Q, QDot, QDDot, tau_inv);
//    std::cout << "tau_inv :\n" << tau_inv << std::endl;

//    std::vector<s2mMuscleStateDynamics> State(m3.nbMuscleTotal());
//    for (unsigned int i = 0; i<m3.nbMuscleTotal(); ++i){
//        State[i].setActivation(0.3925);
//    }
//    //s2mTau tau_inv = m_model.muscularJointTorque(m_model, m_State, true, &m_Q, &m_Qdot);
//    s2mTau tau(m3.nbTau());
//    tau = m3.muscularJointTorque(m3, State, true, &Q, &QDot);
//    std::cout << "tau generated by muscle:\n" << tau << std::endl;

//    unsigned int nbmuscle = m3.nbMuscleTotal();
//    s2mVector activ(nbmuscle);
//    for (unsigned int i=0; i < nbmuscle ; i++){
//        activ[i] = 0.02;
//    }

//    s2mVector a(m3.nbDof());
//    for (unsigned int i=0; i < m3.nbDof() ; i++){
//        a[i] = -9.81;
//    }
//    s2mTau tau(a);
//    s2mStaticOptimization optim(m3, Q, QDot, tau, state);
//    optim.optimize();

//    std::cout << "Q:\n" << Q << std::endl;
//    std::cout << "QDot:\n" << QDot << std::endl;
//    std::cout << "QDDot:\n" << QDDot << std::endl;
//    std::cout << "Tau:\n" << tau << std::endl;



//    s2mMusculoSkeletalModel m3("pyomecaman.bioMod");

//    biorbd::utils::GenCoord q;


//    biorbd::utils::GenCoord Q(m3);
//    biorbd::utils::GenCoord QDDot(m3);
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
