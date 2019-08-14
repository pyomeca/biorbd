#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP
#define BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP

#include "biorbdConfig.h"
#include "RigidBody/KalmanRecons.h"


namespace biorbd {
namespace rigidbody {
class Markers;
class NodeBone;

class BIORBD_API KalmanReconsMarkers : public biorbd::rigidbody::KalmanRecons
{
public:

    // Constructeur
    KalmanReconsMarkers(
            biorbd::Model&,
            KalmanRecons::KalmanParam = KalmanRecons::KalmanParam());
    virtual ~KalmanReconsMarkers();

    // Reconstruction d'un frame
    virtual void reconstructFrame(
            biorbd::Model &m,
            const biorbd::rigidbody::Markers &Tobs,
            biorbd::utils::GenCoord *Q,
            biorbd::utils::GenCoord *Qdot,
            biorbd::utils::GenCoord *Qddot,
            bool removeAxes=true);
    virtual void reconstructFrame(
            biorbd::Model &m,
            const std::vector<biorbd::rigidbody::NodeBone> &Tobs,
            biorbd::utils::GenCoord *Q,
            biorbd::utils::GenCoord *Qdot,
            biorbd::utils::GenCoord *Qddot,
            bool removeAxes=true);
    virtual void reconstructFrame(
            biorbd::Model &m,
            const Eigen::VectorXd &Tobs,
            biorbd::utils::GenCoord *Q = nullptr,
            biorbd::utils::GenCoord *Qdot = nullptr,
            biorbd::utils::GenCoord *Qddot = nullptr,
            bool removeAxes=true); // Faire la reconstruction cinematique
    virtual void reconstructFrame();

    bool first();

protected:
    virtual void initialize();
    virtual void manageOcclusionDuringIteration(
            biorbd::utils::Matrix&,
            Eigen::VectorXd &measure,
            const std::vector<unsigned int> &occlusion);
    biorbd::utils::Matrix m_PpInitial; // Se souvenir de Pp inital
    bool m_firstIteration;
};

}}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP
