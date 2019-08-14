#ifndef BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
#define BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd { namespace muscles {

class BIORBD_API ForceFromOrigin : public biorbd::muscles::Force
{
    public:
        ForceFromOrigin();
        ForceFromOrigin(double x, double y, double z);
        ForceFromOrigin(const Eigen::Vector3d& force);
        ForceFromOrigin(
                const biorbd::muscles::Geometry& geo,
                double force);
        virtual ~ForceFromOrigin();

        // Get et set
        void setForce(const biorbd::muscles::Geometry& geo, double force);

};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
