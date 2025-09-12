#ifndef BIORBD_ALL_H
#define BIORBD_ALL_H

#include "biorbdConfig.h"

#include <rbdl/rbdl.h>

#include "BiorbdModel.h"
#include "ModelReader.h"
#include "ModelWriter.h"
#include "RigidBody/all.h"
#include "Utils/all.h"
#if defined(MODULE_ACTUATORS) || defined(MODULE_MUSCLES) || \
    defined(MODULE_PASSIVE_TORQUES) || defined(MODULE_LIGAMENTS)
#include "InternalForces/all.h"
#endif

#ifdef MODULE_ACTUATORS
#include "InternalForces/Actuators/all.h"
#endif

#ifdef MODULE_MUSCLES
#include "InternalForces/Muscles/all.h"
#endif

#ifdef MODULE_PASSIVE_TORQUES
#include "InternalForces/PassiveTorques/all.h"
#endif

#ifdef MODULE_LIGAMENTS
#include "InternalForces/Ligaments/all.h"
#endif

#ifdef BIORBD_USE_CASADI_MATH
#include "Utils/CasadiExpand.h"
#endif

#endif  // BIORBD_ALL_H
