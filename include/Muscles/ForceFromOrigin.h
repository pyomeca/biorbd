#ifndef BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
#define BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd {
namespace muscles {

    ///
    /// \brief Class ForceFromOrigin
    ///
class BIORBD_API ForceFromOrigin : public biorbd::muscles::Force
{
    public:
        ///
        /// \brief Construct a force from origin
        /// \param x Position of the force on the x axis (default:0)
        /// \param y Position of the force on the y axis (default: 0)
        /// \param z Position of the force on the z axis (default: 0)
        ///
        ForceFromOrigin(
                double x = 0,
                double y = 0,
                double z = 0);
        ///
        /// \brief Construct a force from origin from an Eigen matrix of force
        /// \param other The eigen matrix of force
        ///
        template<typename OtherDerived> ForceFromOrigin(const Eigen::MatrixBase<OtherDerived>& other) :
            biorbd::muscles::Force(other){}

        ///
        /// \brief Construct a force from origin
        /// \param geo Geometry of the muscle
        /// \param vectorNorm The norm of the vector
        ///
        ForceFromOrigin(
                const biorbd::muscles::Geometry& geo,
                double vectorNorm);

        ///
        /// \brief Deep copy of the force from origin
        /// \return A deep copy of the force from origin
        ///
        biorbd::muscles::ForceFromOrigin DeepCopy() const;

        ///
        /// \brief Deep copy of another force from origin
        /// \param other The force from origin to copy
        ///
        void DeepCopy(const biorbd::muscles::ForceFromOrigin& other);

        // Get et set


    ///
    /// \brief Set the force from muscle geometry
    /// \param geo THe muscle geometry
    /// \param vectorNorm The norm of the vector
    ///
        virtual void setForceFromMuscleGeometry(
                const biorbd::muscles::Geometry& geo,
                double vectorNorm);
        ///
        /// \brief Equal operator to be used with an Eigen matrix to construct force from origin
        /// \param other The eigen matrix of force from origin
        /// \return The force from origin
        ///
        template<typename OtherDerived>
            biorbd::muscles::ForceFromOrigin& operator=(const Eigen::MatrixBase <OtherDerived>& other){
                this->biorbd::muscles::Force::operator=(other);
                return *this;
            }
};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
