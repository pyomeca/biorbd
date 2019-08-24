#ifndef BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
#define BIORBD_MUSCLES_FORCE_FROM_INSERTION_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd {
namespace muscles {

class BIORBD_API ForceFromInsertion : public biorbd::muscles::Force
{
public:
    ForceFromInsertion(
            double x = 0,
            double y = 0,
            double z = 0);
    template<typename OtherDerived> ForceFromInsertion(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::muscles::Force(other){}
    ForceFromInsertion(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);
    virtual ~ForceFromInsertion();

    // Get et set
    virtual void setForceFromMuscleGeometry(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);

    template<typename OtherDerived>
        biorbd::muscles::ForceFromInsertion& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::muscles::Force::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
