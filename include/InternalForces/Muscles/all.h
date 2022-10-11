#ifndef BIORBD_MUSCLES_ALL_H
#define BIORBD_MUSCLES_ALL_H

#include "biorbdConfig.h"

#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/Compound.h"
#include "InternalForces/Muscles/FatigueModel.h"
#include "InternalForces/Muscles/FatigueDynamicState.h"
#include "InternalForces/Muscles/FatigueDynamicStateXia.h"
#include "InternalForces/Muscles/FatigueParameters.h"
#include "InternalForces/Muscles/FatigueState.h"
#include "InternalForces/Muscles/Geometry.h"
#include "InternalForces/Muscles/HillThelenActiveOnlyType.h"
#include "InternalForces/Muscles/HillDeGrooteActiveOnlyType.h"
#include "InternalForces/Muscles/HillThelenType.h"
#include "InternalForces/Muscles/HillDeGrooteType.h"
#include "InternalForces/Muscles/HillDeGrooteTypeFatigable.h"
#include "InternalForces/Muscles/HillThelenTypeFatigable.h"
#include "InternalForces/Muscles/HillType.h"
#include "InternalForces/Muscles/IdealizedActuator.h"
#include "InternalForces/Muscles/Muscle.h"
#include "InternalForces/Muscles/MuscleGroup.h"
#include "InternalForces/Muscles/Muscles.h"
#include "InternalForces/Muscles/MusclesEnums.h"
#include "InternalForces/Muscles/State.h"
#include "InternalForces/Muscles/StateDynamics.h"
#include "InternalForces/Muscles/StateDynamicsBuchanan.h"

#ifdef MODULE_STATIC_OPTIM
    #include "InternalForces/Muscles/StaticOptimization.h"
    #include "InternalForces/Muscles/StaticOptimizationIpopt.h"
    #include "InternalForces/Muscles/StaticOptimizationIpoptLinearized.h"
#endif

#endif // BIORBD_MUSCLES_ALL_H

