    #ifndef BIORBD_MUSCLES_COMPOUND_H
#define BIORBD_MUSCLES_COMPOUND_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "Muscles/MusclesEnums.h"

namespace biorbd {
namespace utils {
class String;
class Node3d;
}

namespace rigidbody {
class Joints;
class GeneralizedCoordinates;
}

namespace muscles {
class Force;
class Characteristics;
class PathChangers;
class StateDynamics;
///
/// \brief Class compound
///
class BIORBD_API Compound
{
public:
    ///
    /// \brief Construct compound
    ///
    Compound();
    ///
    /// \brief Construct compound
    /// \param name Name of the compound
    ///
    Compound(
            const biorbd::utils::String &name);
    ///
    /// \brief Construct compound
    /// \param name Name of the compound
    /// \param wrap Path changer TODO?
    ///
    Compound(
            const biorbd::utils::String &name,
            const biorbd::muscles::PathChangers&wrap);
    ///
    /// \brief Construct compound
    /// \param muscle Muscle to which it is associated
    ///
    Compound(
            const biorbd::muscles::Compound& muscle);

    ///
    /// \brief Construct compound
    /// \param muscle Muscle to which it is associated (pointer)
    ///
    Compound(
            const std::shared_ptr<biorbd::muscles::Compound> muscle);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Compound();

    ///
    /// \brief Deep copy of a compound
    /// \param other Compound to copy
    ///
    void DeepCopy(const biorbd::muscles::Compound& other);

    ///
    /// \brief Return the name of the compound
    /// \return The name of the compound
    ///
    const biorbd::utils::String& name() const;
    ///
    /// \brief Set name to a compound
    /// \param name Name of the compound
    ///
    void setName(const biorbd::utils::String& name);

    ///
    /// \brief Return the type of the compound
    /// \return The type of the compound
    ///
    biorbd::muscles::MUSCLE_TYPE type() const;

    // Wrapping object

    ///
    /// \brief Return the path changer TODO?
    /// \return The path changer
    ///
    const biorbd::muscles::PathChangers& pathChanger();

    /// 
    /// \brief Add a wrapping object
    /// \param w Wrap to add
    ///
    void addPathObject(biorbd::utils::Node3d &w);

    ///
    /// \brief Return the last computed muscle force
    /// \return The last computed muscle force
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force();

    ///
    /// \brief Return the computed forces from the EMG TODO ?
    /// \param emg EMG data
    /// \return The computed forces from the EMG TODO?
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(const biorbd::muscles::StateDynamics& emg) = 0;

    ///
    /// \brief Return the computed force from EMG
    /// \param model The model
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param emg EMG data
    /// \param updateKin Update kinematics (default: 2)
    /// \return The computed force from EMG
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2) = 0;

    ///
    /// \brief Return the computed force from EMG
    /// \param model The model
    /// \param Q The position variables of the model
    /// \param emg EMG data
    /// \param updateKin Update kinematics (default: 2)
    /// \return The computed force from EMG
    ///
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2) = 0;

protected:
    std::shared_ptr<biorbd::utils::String> m_name; ///< The name of the compound
    std::shared_ptr<biorbd::muscles::MUSCLE_TYPE> m_type; ///< The type of the compound
    std::shared_ptr<biorbd::muscles::PathChangers> m_pathChanger; ///< THe path changer
    std::shared_ptr<std::vector<std::shared_ptr<biorbd::muscles::Force>>> m_force; ///< The force

    ///
    /// \brief Set the type
    ///
    virtual void setType()=0;

};

}}

#endif // BIORBD_MUSCLES_COMPOUND_H
