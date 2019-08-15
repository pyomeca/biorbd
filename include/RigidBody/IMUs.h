#ifndef BIORBD_RIGIDBODY_IMUS_H
#define BIORBD_RIGIDBODY_IMUS_H

#include <vector>
#include "biorbdConfig.h"
#include "RigidBody/Patch.h"
#include "Utils/Attitude.h"
#include "Utils/String.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class Matrix;
}

namespace rigidbody {
class Joints;
class GeneralizedCoordinates;
class IMU;

class BIORBD_API IMUs
{
public:
    IMUs();
    virtual ~IMUs();

    // Set and get
    void addIMU(
            const biorbd::utils::Attitude &pos = biorbd::utils::Attitude(),
            const biorbd::utils::String &name = "",
            const biorbd::utils::String &parentName = "",
            const bool &technical = true,
            const bool &anatomical = false,
            const int &id = -1); // Ajouter un nouveau marker
    unsigned int nIMUs() const; // Retourne le nombre de marqueurs

    std::vector<biorbd::utils::String> IMUsNames();
    std::vector<biorbd::utils::String> technicalIMUsNames();
    std::vector<biorbd::utils::String> anatomicalIMUsNames();

    std::vector<biorbd::rigidbody::IMU> IMU(); // Retour d'un STL vector de tous les IMU
    std::vector<biorbd::rigidbody::IMU> IMU(biorbd::rigidbody::Joints& m, unsigned int idx); // Retour d'un STL vector de tous les IMU d'un segment idx
    const biorbd::rigidbody::IMU& IMU(const unsigned int&); // Retour d'un IMU ind idx

    std::vector<biorbd::rigidbody::IMU> IMU(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const bool &updateKin = true); // Retour d'un STL vector de tous les IMUs
    biorbd::rigidbody::IMU IMU(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const unsigned int& idx,
            const bool &updateKin = true); // Retour d'un IMU ind idx
    std::vector<biorbd::rigidbody::IMU> segmentIMU(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const unsigned int& idx,
            const bool &updateKin = true); // Retour d'un STL vector de tous les IMUs d'un segment


    unsigned int nTechIMUs(); // Retourne le nombre de marqueurs techniques
    unsigned int nAnatIMUs(); // Retourne le nombre de marqueurs anatomiques
    std::vector<biorbd::rigidbody::IMU> technicalIMU(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs
    std::vector<biorbd::rigidbody::IMU> technicalIMU(); // Retour d'un STL vector de tous les IMUs
    std::vector<biorbd::rigidbody::IMU> anatomicalIMU(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs
    std::vector<biorbd::rigidbody::IMU> anatomicalIMU(); // Retour d'un STL vector de tous les IMUs




    std::vector<biorbd::utils::Matrix> IMUJacobian(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const bool &updateKin = true); // Retourne la jacobienne des Tags
    std::vector<biorbd::utils::Matrix> TechnicalIMUJacobian(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const bool &updateKin = true); // Retourne la jacobienne des Tags pour les marqueurs techniques

protected:
    std::vector<biorbd::utils::Matrix> IMUJacobian(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const bool &updateKin,
            bool lookForTechnical); // Retourne la jacobienne des Tags

    std::vector <biorbd::rigidbody::IMU> m_IMUs;

};

}}

#endif // BIORBD_RIGIDBODY_IMUS_H
