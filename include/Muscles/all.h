#ifndef BIORBD_MUSCLES_ALL_H
#define BIORBD_MUSCLES_ALL_H

#include "biorbdConfig.h"

#include "Muscles/Caracteristics.h"
#include "Muscles/Compound.h"
#include "Muscles/Fatigable.h"
#include "Muscles/FatigueDynamicState.h"
#include "Muscles/FatigueDynamicStateXia.h"
#include "Muscles/FatigueParameters.h"
#include "Muscles/FatigueState.h"
#include "Muscles/Force.h"
#include "Muscles/ForceFromInsertion.h"
#include "Muscles/ForceFromOrigin.h"
#include "Muscles/Geometry.h"
#include "Muscles/HillThelenType.h"
#include "Muscles/HillThelenTypeFatigable.h"
#include "Muscles/HillType.h"
#include "Muscles/IdealizedActuator.h"
#include "Muscles/Muscle.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/Muscles.h"
#include "Muscles/MusclesEnums.h"
#include "Muscles/PathChangers.h"
#include "Muscles/State.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/StateDynamicsBuchanan.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/WrappingCylinder.h"
#include "Muscles/WrappingObject.h"
#include "Muscles/WrappingSphere.h"

#ifndef SKIP_STATIC_OPTIM
#include "Muscles/StaticOptimization.h"
#include "Muscles/StaticOptimizationIpopt.h"
#include "Muscles/StaticOptimizationIpoptLinearized.h"
#endif

#endif // BIORBD_MUSCLES_ALL_H

