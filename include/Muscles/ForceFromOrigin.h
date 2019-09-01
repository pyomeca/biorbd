#ifndef BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
#define BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd {
namespace muscles {

class BIORBD_API ForceFromOrigin : public biorbd::muscles::Force
{
    public:
        ForceFromOrigin(
                double x = 0,
                double y = 0,
                double z = 0);
        template<typename OtherDerived> ForceFromOrigin(const Eigen::MatrixBase<OtherDerived>& other) :
            biorbd::muscles::Force(other){}
        ForceFromOrigin(
                const biorbd::muscles::Geometry& geo,
                double vectorNorm);
        biorbd::muscles::ForceFromOrigin DeepCopy() const;
        void DeepCopy(const biorbd::muscles::ForceFromOrigin& other);

        // Get et set
        virtual void setForceFromMuscleGeometry(
                const biorbd::muscles::Geometry& geo,
                double vectorNorm);

        template<typename OtherDerived>
            biorbd::muscles::ForceFromOrigin& operator=(const Eigen::MatrixBase <OtherDerived>& other){
                this->biorbd::muscles::Force::operator=(other);
                return *this;
            }
};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
