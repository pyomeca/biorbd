#ifndef BIORBD_RIGIDBODY_IMUS_H
#define BIORBD_RIGIDBODY_IMUS_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Matrix;
class RotoTransNode;
}

namespace rigidbody {
class GeneralizedCoordinates;
class IMU;

class BIORBD_API IMUs
{
public:
    ///
    /// \brief Create inertial measurement units data
    ///
    IMUs();

    ///
    /// \brief Create inertial measurement units data
    /// \param other TODO: ?
    ///
    IMUs(const biorbd::rigidbody::IMUs& other);
    
    ///
    /// \brief Destroy the class properly
    ///
    virtual ~IMUs();
    
    ///
    /// \brief Deep copy of the inertial measurement units data
    /// \return A copy of the inertial measurement units data
    ///
    biorbd::rigidbody::IMUs DeepCopy() const;
    ///
    /// \brief Deep copy the inertial measurement units data
    ///
    void DeepCopy(const biorbd::rigidbody::IMUs& other);

    // Set and get

    ///
    /// \brief Add a new inertial measurement unit (IMU)
    /// \param technical True if the IMU is technical
    /// \param anatomical True if the IMU is anatomical
    ///
    void addIMU(
            bool technical = true,
            bool anatomical = true); 

    ///
    /// \brief Add a new IMU to the existing pool of IMUs
    /// \param RotoTrans The position 
    /// \param technical True if the IMU is technical
    /// \param anatomical True if the IMU is anatomical
    ///
    void addIMU(
            const biorbd::utils::RotoTransNode &RotoTrans,
            bool technical = true,
            bool anatomical = true); 

    ///
    /// \brief Return the number of inertial measurement units (IMUs)
    /// \return The number of inertial measurement units
    ///
    unsigned int nbIMUs() const;

    ///
    /// \brief Return the names of the inertial measurement units (IMUs)
    /// \return The names of the inertial measurements units (IMUs)
    ///
    std::vector<biorbd::utils::String> IMUsNames();

    ///
    /// \brief Return the names of the technical inertial measurement units (IMUs)
    /// \return The names of the technical intertial measurement units
    ///
    std::vector<biorbd::utils::String> technicalIMUsNames();

    ///
    /// \brief Return the names of the anatomical inertial measurement units (IMUs)
    /// \return The names of the anatomical inertial measurement units
    ///
    std::vector<biorbd::utils::String> anatomicalIMUsNames(); 


    ///
    /// \brief Return STL vector of all the makers in the local reference
    /// \return An STL vector of all the inertial measurement units
    ///
    const std::vector<biorbd::rigidbody::IMU>& IMU() const;

    ///
    /// \brief Return STL vector of all the inertial measurement units (IMU) of an idx segment
    /// \param segmentName The name of the segment
    /// \return An STL vector of all the inertial measurement units (IMU) of an idx segment
    ///
    std::vector<biorbd::rigidbody::IMU> IMU(const biorbd::utils::String& segmentName); 

    ///
    /// \brief Return inertial measurement unit at a specified idx
    /// \param i Specific idx
    /// \return Inertial measurement unit at idx i
    ///
    const biorbd::rigidbody::IMU& IMU(unsigned int i); 


    ///
    /// \brief Return an STL vector of all the inertial measurement units (IMUs) at the position given by Q
    /// \param Q State vector of the internal joints
    /// \param updatekin True by default
    /// \return STL vector of all the inertial measurement units (IMUs) at the position given by Q
    ///
    std::vector<biorbd::rigidbody::IMU> IMU(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true);

    ///
    /// \brief Return an IMU at the position given by Q
    /// \param Q State vector of the internal joints
    /// \param idx Specific index
    /// \param updatekin True by default
    /// \return an inertial measurement unit
    ///
    biorbd::rigidbody::IMU IMU(
            const biorbd::rigidbody::GeneralizedCoordinates&Q,
            unsigned int  idx,
            bool updateKin = true);

    ///
    /// \brief Return an STL vector of all the inertial measurement units (IMUs) of a specific segment
    /// \param Q State vector of the internal joints
    /// \param idx Segment index
    /// \param updateKin True by default
    /// \return STL of all the inertial measurement units of specific segment with idx index
    ///
    std::vector<biorbd::rigidbody::IMU> segmentIMU(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            unsigned int  idx,
            bool updateKin = true); 

    ///
    /// \brief Return the number of technical IMUs
    /// \return The number of technical IMUs
    ///
    unsigned int nbTechIMUs();

    ///
    /// \brief Return the number of anatomical IMUs
    /// \return The number of anatomical IMUs
    ///
    unsigned int nbAnatIMUs();

    ///
    /// \brief Return an STL vector of all the technical IMUs
    /// \param Q State vector of the internal joints
    /// \param updateKin True by default
    /// \return STL vector of all the technical IMUs
    ///
    std::vector<biorbd::rigidbody::IMU> technicalIMU(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true);

    ///
    /// \brief Return STL vector of all the technical IMUs in the local reference
    /// \return STL vector of all the technical IMUs
    ///
    std::vector<biorbd::rigidbody::IMU> technicalIMU();

    ///
    /// \brief Return STL vector of anatomical IMUs 
    /// \param Q State vector of the internal joints
    /// \param updateKin True by default
    /// \return STL vector of anatomical IMUs
    ///
    std::vector<biorbd::rigidbody::IMU> anatomicalIMU(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true);

    ///
    /// \brief Return STL vector of anatomical IMUs in the local reference
    /// \return STL vector of anatomical IMUs
    ///
    std::vector<biorbd::rigidbody::IMU> anatomicalIMU(); 

    ///
    /// \brief Return the Jacobian of the IMUs (TODO: or of the markers?)
    /// \param Q State vector of the internal joints
    /// \param updateKin True by default
    /// \return The Jacobien of the IMUs
    ///
    std::vector<biorbd::utils::Matrix> IMUJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retourne la jacobienne des Markers

    ///
    /// \brief Return the Jacobian of the technical IMUs
    /// \param Q State vector of the internal joints
    /// \param updateKin True by default
    /// \return The Jacobian of the technical IMUs
    ///
    std::vector<biorbd::utils::Matrix> TechnicalIMUJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retourne la jacobienne des Markers pour les marqueurs techniques

protected:

    ///
    /// \brief Return the Jacobian of the IMUs (Markers)
    /// \param Q State vector of the internal joints
    /// \param updateKin 
    /// \lookForTechnical Check if there are any technical IMUs
    ///
    std::vector<biorbd::utils::Matrix> IMUJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin,
            bool lookForTechnical);

    std::shared_ptr<std::vector<biorbd::rigidbody::IMU>> m_IMUs; ///< Inertial Measurement Units

};

}}

#endif // BIORBD_RIGIDBODY_IMUS_H
