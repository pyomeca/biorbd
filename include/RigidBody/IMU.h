#ifndef BIORBD_RIGIDBODY_IMU_H
#define BIORBD_RIGIDBODY_IMU_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/RotoTransNode.h"

namespace biorbd {
namespace utils {
class String;
}

namespace rigidbody {

///
/// \brief Class IMU
///
class BIORBD_API IMU : public biorbd::utils::RotoTransNode
{ 
public:

    ///
    /// \brief Construct inertial measurement unit data
    /// \param isTechnical True if the IMU is a technical IMU
    /// \param isAnatomical True if the IMU is an anatomical IMU
    ///
    IMU(
            bool isTechnical = true, 
            bool isAnatomical = true);

    ///
    /// \brief Construct inertial measurement unit data
    /// \param RotoTrans TODO: ? The position
    /// \param isTechnical True if the IMU is a technical IMU
    /// \param isAnatomical True if the IMU is an anatomical IMU
    ///
    IMU(
            const biorbd::utils::RotoTransNode& RotoTrans, 
            bool isTechnical = true, 
            bool isAnatomical = true); 

    ///
    /// \brief Construct inertial measurement unit data
    /// \param other
    ///
    template<typename OtherDerived> IMU(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::RotoTransNode(other){}

    ///
    /// \brief Deep copy of the IMU data
    /// \return Copy of the IMU data
    ///
    biorbd::rigidbody::IMU DeepCopy() const;

    ///
    /// \brief Deep copy if the IMU data
    /// \param other The IMU to copy
    ///
    void DeepCopy(const biorbd::rigidbody::IMU& other);

    // Get and Set

    ///
    /// \brief Return if the IMU is technical
    /// \return True or False
    ///
    bool isTechnical() const;

    ///
    /// \brief Return if the IMU is anatomical
    /// \return True or False
    ///
    bool isAnatomical() const;

    ///
    /// \brief TODO:?
    /// \param other TODO:
    /// \return TODO:
    template<typename OtherDerived>
        biorbd::rigidbody::IMU& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::RotoTransNode::operator=(other);
            return *this;
        }

protected:
    std::shared_ptr<bool> m_technical; ///< If a IMU is a technical IMU
    std::shared_ptr<bool> m_anatomical; ///< It IMU is a anatomical IMU

};

}}

#endif // BIORBD_RIGIDBODY_IMU_H
