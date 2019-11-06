#ifndef BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
#define BIORBD_MUSCLES_FORCE_FROM_INSERTION_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd {
namespace muscles {

    ///
    /// \brief Class ForceFromInsertion
    ///
class BIORBD_API ForceFromInsertion : public biorbd::muscles::Force
{
public:
    ///
    /// \brief Construct force from insertion
    /// \param x Position on the x axis (default: 0)
    /// \param y Position on the y axis (default: 0)
    /// \param z Position on the z axis (default: 0)
    ///
    ForceFromInsertion(
            double x = 0,
            double y = 0,
            double z = 0);

    ///
    /// \brief Construct force from insertion from Eiggen matrix
    /// \param other The Eigen force matrix
    ///
    template<typename OtherDerived> ForceFromInsertion(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::muscles::Force(other){}

    ///
    /// \brief Construct force from insertion
    /// \param geo The muscle geometry
    /// \param vectorNorm The norm of the vector
    ///
    ForceFromInsertion(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);

    ///
    /// \brief Deep copy of the force from insertion
    /// \return Deep copy ofo the force from insertion
    ///
    biorbd::muscles::ForceFromInsertion DeepCopy() const;

    ///
    /// \brief Deep copy of the force from insertion from another force from insertion
    /// \param other The other force from insertion
    ///
    void DeepCopy(const biorbd::muscles::ForceFromInsertion& other);

    // Get et set

    ///
    /// \brief Set the force from muscle geometry
    /// \param geo THe muscle geometry
    /// \parm vectorNorm The norm of the vector
    ///
    virtual void setForceFromMuscleGeometry(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);

    ///
    /// \brief Equal operator to be used with an Eigen matrix to construct force from insertion
    /// \param other The eigen matrix of force from insertion
    /// \return The force from insertion
    ///
    template<typename OtherDerived>
        biorbd::muscles::ForceFromInsertion& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::muscles::Force::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
