#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP
#define BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP

#include "biorbdConfig.h"
#include "RigidBody/KalmanRecons.h"


namespace biorbd {
namespace rigidbody {
class Markers;
class NodeSegment;

///
/// \brief Class KalmanReconsMarkers that includes class KalmanRecons
///
class BIORBD_API KalmanReconsMarkers : public biorbd::rigidbody::KalmanRecons
{
public:

    // Constructor

    /// 
    /// \brief Initialize the Kalman filter and Kalman reconstruction for Markers data
    ///
    KalmanReconsMarkers();

    /// 
    /// \brief Initialize the Kalman filter and Kalman reconstruction for Markers data
    /// \param model The model
    /// \param params The Kalman filter parameters
    ///
    KalmanReconsMarkers(
            biorbd::Model& model,
            biorbd::rigidbody::KalmanRecons::KalmanRecons::KalmanParam params = biorbd::rigidbody::KalmanRecons::KalmanRecons::KalmanParam());

    /// 
    /// \brief Deep copy of the Kalman reconstruction
    /// \return Copy of the Kalman reconstruction
    ///
    biorbd::rigidbody::KalmanReconsMarkers DeepCopy() const;

    /// 
    /// \brief Deep copy of the Kalman reconstruction
    /// \param other The Kalman reconstruction to copy
    ///
    void DeepCopy(const biorbd::rigidbody::KalmanReconsMarkers& other);

 
    ///
    /// \brief Reconstruct the kinematics from markers data
    /// \param model The model
    /// \param Tobs The observed markers
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \param removeAxes (default=True)
    ///
    virtual void reconstructFrame(
            biorbd::Model &model,
            const biorbd::rigidbody::Markers &Tobs,
            biorbd::rigidbody::GeneralizedCoordinates *Q,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot,
            bool removeAxes=true);

    ///
    /// \brief Reconstruct the kinematics from markers data
    /// \param model The model
    /// \param Tobs The observed markers
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \param removeAxes (default=True)
    ///
    virtual void reconstructFrame(
            biorbd::Model &model,
            const std::vector<biorbd::rigidbody::NodeSegment> &Tobs,
            biorbd::rigidbody::GeneralizedCoordinates *Q,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot,
            bool removeAxes=true);

    ///
    /// \brief Reconstruct the kinematics from markers data
    /// \param model The model
    /// \param Tobs The observed markers (all already in a big vector)
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    /// \param removeAxes (default=True)
    ///
    virtual void reconstructFrame(
            biorbd::Model &model,
            const biorbd::utils::Vector &Tobs,
            biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr,
            bool removeAxes=true);

    /// 
    /// \brief Error message if no input in function
    ///
    virtual void reconstructFrame();

    ///
    /// \brief To know if the first iteration was done
    /// \return True or False
    ///
    bool first();

protected:
    ///
    /// \brief To initialize the Kalman filter
    ///
    virtual void initialize();


    ///
    /// \brief Manage the occlusion during the iteration
    /// \param InvTp The inverse of the Tp matrix
    /// \param measure The measure
    /// \param occlusion The occlusion TODO
    ///
    virtual void manageOcclusionDuringIteration(
            biorbd::utils::Matrix& InvTp,
            biorbd::utils::Vector &measure,
            const std::vector<unsigned int> &occlusion);
    std::shared_ptr<biorbd::utils::Matrix> m_PpInitial; ///< Initial Pp matrix
    std::shared_ptr<bool> m_firstIteration; ///< To know if first iteration was done (True or False)
};

}}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP
