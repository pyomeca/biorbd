#include <iostream>
#include "s2mMusculoSkeletalModel.h"
#include "s2mStaticOptimization.h"
#include <memory>
#include "IpoptTestMain.h"
#include <cstdlib>
#include <random>
#define BUILD_SANDBOX

#ifdef BUILD_SANDBOX


int main()
{

    s2mString path("conv-arm26.bioMod"); // Model path
    s2mMusculoSkeletalModel model(path);
    s2mGenCoord Q(model);
    s2mGenCoord QDot(model);
    s2mGenCoord QDDot(model);
    for (unsigned i = 0; i<model.nbQ(); i++){
        Q[i] = 0 ;
        QDot[i] = 0;
        QDDot[i] = 0;
    }
    model.updateMuscles(model, Q, QDot, true);

    s2mMuscleStateActual emg(0, 1);
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



