#ifndef BIORBD_ALL_H
#define BIORBD_ALL_H

#include <rbdl/rbdl.h>

#include "biorbdConfig.h"
#include "BiorbdModel.h"
#include "ModelReader.h"
#include "ModelWriter.h"

#include "Utils/all.h"
#include "RigidBody/all.h"
#if defined(MODULE_ACTUATORS) || defined(MODULE_MUSCLES)
#include "InternalForces/all.h"
#endif

#ifdef MODULE_ACTUATORS
#include "InternalForces/Actuators/all.h"
#endif

#ifdef MODULE_MUSCLES
#include "InternalForces/Muscles/all.h"
#include "InternalForces/PassiveTorques/all.h"

#endif


#ifdef BIORBD_USE_CASADI_MATH
#include "Utils/CasadiExpand.h"
#endif

#endif // BIORBD_ALL_H

