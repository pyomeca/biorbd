#ifndef BIORBD_MUSCLES_COMPOUND_H
#define BIORBD_MUSCLES_COMPOUND_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
class Vector3d;
}

namespace rigidbody
{
class Joints;
class GeneralizedCoordinates;
class GeneralizedVelocity;
}
namespace internalforce
{
class PathModifiers;

///
/// \brief Class compound is a very generic definition of what a muscle is. It should be the base class of every muscles
///
class BIORBD_API Compound
{
public:
    ///
    /// \brief Construct muscle compound
    ///
    Compound();

    ///
    /// \brief Construct compound
    /// \param name Name of the compound
    ///
    Compound(
        const utils::String &name);

    ///
    /// \brief Construct compound
    /// \param name Name of the compound
    /// \param pathModifiers The set of path modifiers
    ///
    Compound(
        const utils::String& name,
        const internalforce::PathModifiers& pathModifiers);
    ///
    /// \brief Construct compound from another muscle
    /// \param other The muscle to shallow copy
    ///
    Compound(
        const Compound& other);

    ///
    /// \brief Construct compound from another muscle
    /// \param other The muscle to shallow copy
    ///
    Compound(
        const std::shared_ptr<Compound> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Compound();

    ///
    /// \brief Deep copy of a compound
    /// \param other Compound to copy
    ///
    void DeepCopy(
        const Compound& other);

    ///
    /// \brief Set the name of a muscle
    /// \param name Name of the muscle
    ///
    void setName(const utils::String& name);

    ///
    /// \brief Return the name of the muscle
    /// \return The name of the muscle
    ///
    const utils::String& name() const;


    // Wrapping object
    ///
    /// \brief Return the path modifier
    /// \return The path modifier
    ///
    const internalforce::PathModifiers& pathModifier();

    ///
    /// \brief Add a path modifier object
    /// \param wrap Position of the object
    ///
    void addPathObject(utils::Vector3d& wrap);

    ///
    /// \brief Return the last computed muscle force norm
    /// \return The last computed muscle force norm
    ///
    virtual const utils::Scalar& force();


protected:
    std::shared_ptr<utils::String> m_name; ///< The name of the muscle
    std::shared_ptr<internalforce::PathModifiers>
    m_pathChanger; ///< The set of path modifiers
    std::shared_ptr<utils::Scalar> m_force; ///< The last computed force

};

}
}

#endif // BIORBD_MUSCLES_COMPOUND_H
