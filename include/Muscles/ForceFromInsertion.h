#ifndef BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
#define BIORBD_MUSCLES_FORCE_FROM_INSERTION_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd {
namespace muscles {

class BIORBD_API ForceFromInsertion : public biorbd::muscles::Force
{
public:
    ForceFromInsertion();
    ForceFromInsertion(double x, double y, double z);
    ForceFromInsertion(const Eigen::Vector3d& force);
    ForceFromInsertion(
            const biorbd::muscles::Geometry& geo,
            double force);
    virtual ~ForceFromInsertion();

    // Get et set
    virtual void setForce(const biorbd::muscles::Geometry& geo, double force);

};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
