#ifndef BIORBD_MUSCLES_WRAPPING_OBJECT_H
#define BIORBD_MUSCLES_WRAPPING_OBJECT_H

#include "biorbdConfig.h"
#include "Utils/Vector3d.h"

namespace biorbd {
namespace utils {
class String;
}

namespace rigidbody {
class Joints;
class GeneralizedCoordinates;
}

namespace muscles {
    ///
    /// \brief Class WrappingObject
    ///
class BIORBD_API WrappingObject : public biorbd::utils::Vector3d
{
public:
    ///
    /// \brief Construct a wrapping object
    ///
    WrappingObject();
    ///
    /// \brief Construct a wrapping object
    /// \param x Position on x axis
    /// \param y Position on y axis
    /// \param z Position on z axis
    ///
    WrappingObject(
            double x,
            double y,
            double z);
    ///
    /// \brief Construct a wrapping object
    /// \param x Position on x axis
    /// \param y Position on y axis
    /// \param z Position on z axis
    /// \param name Name of the node
    /// \param parentName Name of the parent
    ///
    WrappingObject(
            double x,
            double y,
            double z,
            const biorbd::utils::String &name,  
            const biorbd::utils::String &parentName);

    ///
    /// \brief Construct a wrapping object from 3d Node
    /// \param other The 3d node
    ///
    WrappingObject(
            const biorbd::utils::Vector3d& other);
    ///
    /// \brief Construct a wrapping object
    /// \param other Eigen vector
    /// \param name Name of the node
    /// \param parentName Name of the parent
    ///
    WrappingObject(
            const Eigen::Vector3d& other,
            const biorbd::utils::String& name,
            const biorbd::utils::String& parentName);
    ///
    /// \brief Deep copy of the wrapping ibject in another wrapping cylinder
    /// \param other The wrapping object to copy
    ///
    void DeepCopy(const biorbd::muscles::WrappingObject& other);

    ///
    /// \brief This function takes the position of the wrapping and finds the location where muscle 1 and 2 leave the wrapping object
    /// \param rt RotoTrans matrix
    /// \param p1_bone TODO
    /// \param p2_bone TODO
    /// \param p1 TODO
    /// \param p2 TODO
    /// \param muscleLength Length of the muscle (default: nullptr)
    ///
    virtual void wrapPoints(
            const biorbd::utils::RotoTrans& rt,
            const biorbd::utils::Vector3d& p1_bone,
            const biorbd::utils::Vector3d& p2_bone,
            biorbd::utils::Vector3d& p1,
            biorbd::utils::Vector3d& p2,
            double* muscleLength = nullptr) = 0 ; // Premier et dernier points musculaire
    ///
    /// \brief This function takes a model and a position and finds the location where muscle 1 and 2 leave the wrapping object
    /// \param model The model
    /// \param Q The position variables 
    /// \param p1_bone TODO
    /// \param p2_bone TODO
    /// \param p1 TODO
    /// \param p2 TODO 
    /// \param muscleLength Length of the muscle (default: nullptr)
    ///
    virtual void wrapPoints(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::utils::Vector3d& p1_bone,
            const biorbd::utils::Vector3d& p2_bone,
            biorbd::utils::Vector3d& p1,
            biorbd::utils::Vector3d& p2,
            double* muscleLength = nullptr) = 0; // Premier et dernier points musculaire

    ///
    /// \brief This function takes finds the location where muscle 1 and 2 leave the wrapping object (if already computed)
    /// \param p1 TODO
    /// \param p2 TODO
    /// \param muscleLength Length of the muscle (default: nullptr)
    ///
    virtual void wrapPoints(
            biorbd::utils::Vector3d& p1,
            biorbd::utils::Vector3d& p2,
            double* muscleLength = nullptr) = 0; // Assume un appel déja faits
    ///
    /// \brief Return the RotoTrans matrix of the wrapping object
    /// \param model The model
    /// \param Q The position variables
    /// \param updateKin Update kinematics (default: True)
    /// \return The RotoTrans matrix of the wrapping object 
    ///
    virtual const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool updateKin = true) = 0;
    ///
    /// \brief Return the RotoTrans matrix of the wrapping ibject
    /// \return The RotoTrans matrix of the wrapping object
    ///
    const biorbd::utils::RotoTrans& RT() const;

    ///
    /// \brief To be able to use the equal "=" operator to define wrapping object 
    /// \param other The 3d node to define the wrapping object
    biorbd::muscles::WrappingObject& operator=(const biorbd::utils::Vector3d& other){
        this->biorbd::utils::Vector3d::operator=(other);
        return *this;
    }
protected:
    std::shared_ptr<biorbd::utils::RotoTrans> m_RT; ///< RotoTrans matrix
};

}}

#endif // BIORBD_MUSCLES_WRAPPING_OBJECT_H
