#ifndef BIORBD_MUSCLES_FORCE_H
#define BIORBD_MUSCLES_FORCE_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"
#include "Muscles/Geometry.h"

namespace biorbd {
namespace muscles {

class BIORBD_API Force : public Eigen::Vector3d
{
public:
    Force(
            double x = 0,
            double y = 0,
            double z = 0);
    Force(const biorbd::utils::Node& force);
    Force(
            const biorbd::muscles::Geometry& geo,
            double force);

    // Get et set
    double norme() const;
    const Force &directionVector() const;
    virtual void setForce(const biorbd::utils::Node &force);
    virtual void setForce(double x, double y, double z);
    virtual void setForce(const biorbd::muscles::Geometry& geo, double force);
protected:
    void computeNorm();
    double m_force;

};

}}

#endif // BIORBD_MUSCLES_FORCE_H
