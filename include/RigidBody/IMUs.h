#ifndef BIORBD_RIGIDBODY_IMUS_H
#define BIORBD_RIGIDBODY_IMUS_H

#include <vector>
#include "biorbdConfig.h"
#include "RigidBody/Patch.h"
#include "Utils/NodeAttitude.h"
#include "Utils/String.h"
#include "Utils/Node3d.h"

namespace biorbd {
namespace utils {
class Matrix;
}

namespace rigidbody {
class GeneralizedCoordinates;
class IMU;

class BIORBD_API IMUs
{
public:
    IMUs();
    virtual ~IMUs();

    // Set and get
    void addIMU(
            bool technical = true,
            bool anatomical = true); // Add a new IMU
    void addIMU(
            const biorbd::utils::NodeAttitude &attitude,
            bool technical = true,
            bool anatomical = true); // Add a new IMU
    unsigned int nIMUs() const; // Retourne le nombre de IMU

    std::vector<biorbd::utils::String> IMUsNames();
    std::vector<biorbd::utils::String> technicalIMUsNames();
    std::vector<biorbd::utils::String> anatomicalIMUsNames();

    const std::vector<biorbd::rigidbody::IMU>& IMU() const; // Retour d'un STL vector de tous les IMU
    std::vector<biorbd::rigidbody::IMU> IMU(const biorbd::utils::String& segmentName); // Retour d'un STL vector de tous les IMU d'un segment idx
    const biorbd::rigidbody::IMU& IMU(unsigned int ); // Retour d'un IMU ind idx

    std::vector<biorbd::rigidbody::IMU> IMU(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs
    biorbd::rigidbody::IMU IMU(
            const biorbd::rigidbody::GeneralizedCoordinates&,
            unsigned int  idx,
            bool updateKin = true); // Retour d'un IMU ind idx
    std::vector<biorbd::rigidbody::IMU> segmentIMU(
            const biorbd::rigidbody::GeneralizedCoordinates&,
            unsigned int  idx,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs d'un segment


    unsigned int nTechIMUs(); // Retourne le nombre de marqueurs techniques
    unsigned int nAnatIMUs(); // Retourne le nombre de marqueurs anatomiques
    std::vector<biorbd::rigidbody::IMU> technicalIMU(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs
    std::vector<biorbd::rigidbody::IMU> technicalIMU(); // Retour d'un STL vector de tous les IMUs
    std::vector<biorbd::rigidbody::IMU> anatomicalIMU(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs
    std::vector<biorbd::rigidbody::IMU> anatomicalIMU(); // Retour d'un STL vector de tous les IMUs




    std::vector<biorbd::utils::Matrix> IMUJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retourne la jacobienne des Tags
    std::vector<biorbd::utils::Matrix> TechnicalIMUJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retourne la jacobienne des Tags pour les marqueurs techniques

protected:
    std::vector<biorbd::utils::Matrix> IMUJacobian(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin,
            bool lookForTechnical); // Retourne la jacobienne des Tags

    std::vector <biorbd::rigidbody::IMU> m_IMUs;

};

}}

#endif // BIORBD_RIGIDBODY_IMUS_H
