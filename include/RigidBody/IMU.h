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

class BIORBD_API IMU : public biorbd::utils::RotoTransNode
{ 
public:

    ///
    /// \brief Create inertial measurement unit data
    /// \param isTechnical True if the marker is a technical marker
    /// \param isAnatomical True if the marker is an anatomical marker
    ///
    IMU(
            bool isTechnical = true, 
            bool isAnatomical = true);

    ///
    /// \brief Create inertial measurement unit data
    /// \param RotoTrans The position
    /// \param isTechnical True if the marker is a technical marker
    /// \param isAnatomical True if the marker is an anatomical marker
    ///
    IMU(
            const biorbd::utils::RotoTransNode& RotoTrans, /
            bool isTechnical = true, 
            bool isAnatomical = true); 
    template<typename OtherDerived> IMU(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::RotoTransNode(other){}

    ///
    /// \brief Deep copy of the IMU data
    /// \return Copy of the IMU data
    biorbd::rigidbody::IMU DeepCopy() const;

    ///
    /// \brief Deep copy if the IMU data
    /// \param other TODO:?
    ///
    void DeepCopy(const biorbd::rigidbody::IMU& other);

    // Get and Set
    // TODO Inherit isTechnical
    ///
    /// \brief Return if the maker is technical or not
    /// \return True or False
    ///
    bool isTechnical() const;

    ///
    /// \brief Return if the marker is anatomical or not
    /// \return True or False
    ///
    bool isAnatomical() const;
    template<typename OtherDerived>
        biorbd::rigidbody::IMU& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::RotoTransNode::operator=(other);
            return *this;
        }

protected:
    std::shared_ptr<bool> m_technical; // If a marker is a technical marker
    std::shared_ptr<bool> m_anatomical; // It marker is a anatomical marker

};

}}

#endif // BIORBD_RIGIDBODY_IMU_H
