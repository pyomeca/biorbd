#define BIORBD_API_EXPORTS
#include "Muscles/FatigueDynamicStateXia.h"

#include <cmath>
#include "Muscles/Caracteristics.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(
        double active,
        double fatigued,
        double resting) :
    biorbd::muscles::FatigueDynamicState(active,fatigued,resting)
{
    setType();
}

biorbd::muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(const std::shared_ptr<biorbd::muscles::FatigueState> m):
    biorbd::muscles::FatigueDynamicState(m)
{

}

void biorbd::muscles::FatigueDynamicStateXia::timeDerivativeState(const biorbd::muscles::StateDynamics &emg, const biorbd::muscles::Caracteristics &caract){
    // Getting the command
    double targetCommand(emg.activation());
    double command(0);
    if (m_activeFibers < targetCommand){
        if (m_restingFibers > targetCommand - m_activeFibers){
            command = caract.fatigueParameters().developFactor()*(targetCommand - m_activeFibers);
        } else {
            command = caract.fatigueParameters().developFactor()*m_restingFibers;
        }
    } else {
        command = caract.fatigueParameters().recoveryFactor()*(targetCommand - m_activeFibers);
    }

    // Applying the command to the fibers
    m_activeFibersDot = command - caract.fatigueParameters().fatigueRate()*m_activeFibers;
    m_restingFibersDot = -command + caract.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    m_fatiguedFibersDot = caract.fatigueParameters().fatigueRate()*m_activeFibers - caract.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    biorbd::utils::Error::error(fabs(m_activeFibersDot + m_restingFibersDot + m_fatiguedFibersDot) <= 1e-7, "Sum of time derivates of fatigue states must be equal to 0");
}

void biorbd::muscles::FatigueDynamicStateXia::setType()
{
    m_type = "Xia";
}
