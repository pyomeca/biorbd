#ifndef BIORBD_MUSCLES_FORCE_H
#define BIORBD_MUSCLES_FORCE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Muscles/Geometry.h"

namespace biorbd {
namespace muscles {

class BIORBD_API Force : public Eigen::Vector3d
{
public:
    Force();
    Force(double x, double y, double z);
    Force(const Eigen::Vector3d& force);
    Force(
            const biorbd::muscles::Geometry& geo,
            double force);

    // Get et set
    double norme() const;
    const Eigen::Vector3d& directionVector() const;
    virtual void setForce(const Eigen::Vector3d& force);
    virtual void setForce(double x, double y, double z);
    virtual void setForce(const biorbd::muscles::Geometry& geo, double force);
protected:
    void computeNorm();
    double m_force;

};

}}

#endif // BIORBD_MUSCLES_FORCE_H
