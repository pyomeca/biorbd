#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_H
#define BIORBD_RIGIDBODY_KALMAN_RECONS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"


namespace biorbd {
class Model;

namespace utils {
class Matrix;
class Vector;
}

namespace rigidbody {
class GeneralizedCoordinates;

///
/// \brief Class KalmanRecons
///
class BIORBD_API KalmanRecons
{
public:
    ///
    /// \brief Class KalmanParam that holds the Kalman parameters
    ///
    class KalmanParam{
    public:
        /// 
        /// \brief Set the Kalman filter parameters
        /// \param frequency The acquisition frequency (default: 100)
        /// \param noiseFactor The noise factor (default: 1e-10)
        /// \param errorFactor The error factor (default: 1e-5)
        /// 
        KalmanParam(
                double frequency = 100,
                double noiseFactor = 1e-10,
                double errorFactor = 1e-5);

        ///
        /// \brief Return the acquisition frequency
        /// 
        double acquisitionFrequency() const;

        ///
        /// \brief Return the noise factor
        ///
        double noiseFactor() const;

        ///
        /// \brief Return the error factor
        ///
        double errorFactor() const;

    private:
            double m_acquisitionFrequency; ///< The acquisition frequency
            double m_noiseFactor; ///< The noise factor
            double m_errorFactor; ///< The error factor
    };

    // Constructor 

    ///
    /// \brief Kalman reconstruction
    ///
    KalmanRecons();
    
    ///
    /// \brief Kalman reconstruction
    /// \param model The model
    /// \param nMeasure The number of measure TODO?
    /// \param params The Kalman filter parameters
    ///
    KalmanRecons(
            biorbd::Model& model,
            unsigned int nMeasure,
            KalmanParam params = KalmanParam());

    ///
    /// \brief Destroy class properly
    ///
    virtual ~KalmanRecons();

    ///
    /// \brief Deep copy of Kalman reconstruction
    /// \param other The Kalman reconstruction to copy
    ///
    void DeepCopy(const biorbd::rigidbody::KalmanRecons& other);


    ///
    /// \brief Get the state (Q, Qdot, Qddot)
    /// \param Q The generalized coordinates of the model
    /// \param Qdot The generalized velocities of the model
    /// \param Qddot The acceleration variables of the model
    ///
    void getState(
            biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr);

    ///
    /// \brief Set the initial state
    /// \param Q The initial position variables of the model
    /// \param Qdot The initial velocity variables of the model
    /// \param Qddot The initial acceleration variables of the model
    ///
    void setInitState(
            const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr);

    ///
    /// \brief This function needs to be reimplemented the necessary way (with all the arguments). This is to force the user to at least implement this function. However, without argument (particularly the entry data), it is doubtful that the reimplementation will work.
    ///
    virtual void reconstructFrame() = 0;

protected:
    // Internal calculations
    ///
    /// \brief Initialization 
    ///
    virtual void initialize();

    ///
    /// \brief Create the evolution matrix
    /// \param m Number of degrees of freedom
    /// \param n Order of the Taylor development
    /// \param Te 1/ (Acquisition frequency)
    /// \return The evolution matrix
    ///
    biorbd::utils::Matrix evolutionMatrix(
            const unsigned int m,
            unsigned int n,
            double Te); 

    ///
    /// \brief Process the noise matrix
    /// \param nQ TODO?
    /// \param Te 1/ (Acquisition frequency)
    /// \return The noise matrix
    ///
    biorbd::utils::Matrix processNoiseMatrix(
            const unsigned int nQ,
            double Te);


    ///
    /// \brief Matrix of the noise on the measurements
    /// \param nT TODO
    /// \param MN TODO
    /// \return The matrix of the noise on the measurements
    ///
    biorbd::utils::Matrix measurementNoiseMatrix(
            const unsigned int nT,
            double MN);

    ///
    /// \brief Return Pp matrix
    /// \param nQ TODO
    /// \param csnt TODO
    /// \return The Pp matrix
    ///
    biorbd::utils::Matrix initCovariance(
            const unsigned int nQ,
            double csnt);

    ///
    /// \brief Initialize the state
    /// \param nQ TODO
    ///
    biorbd::rigidbody::GeneralizedCoordinates initState(const unsigned int nQ);

    ///
    /// \brief Iterations
    /// \param measure The measure
    /// \param projectedMeasure The projected measure
    /// \param Hessian TODO
    /// \param occlusion TODO
    ///
    void iteration(
            biorbd::utils::Vector measure,
            const biorbd::utils::Vector &projectedMeasure,
            const biorbd::utils::Matrix &Hessian,
            const std::vector<unsigned int> &occlusion = std::vector<unsigned int>());

    ///
    /// \brief Manage the occlusion during the iteration
    /// \param InvTp The inverse of the Tp matrix
    /// \param measure The measurement
    /// \param occlusion TODO
    ///
    virtual void manageOcclusionDuringIteration(
            biorbd::utils::Matrix &InvTp,
            biorbd::utils::Vector &measure,
            const std::vector<unsigned int> &occlusion);

    // Variables attributes
    std::shared_ptr<KalmanParam> m_params; ///< The acquisition frequency
    std::shared_ptr<double> m_Te; ///< Inherent parameter to the frequency
    std::shared_ptr<unsigned int> m_nbDof; ///< Number of DoF, calculated as soon as a model is open
    std::shared_ptr<unsigned int> m_nMeasure; ///< Number of markers 

    // Kalman filter attributes
    std::shared_ptr<biorbd::utils::Vector> m_xp; ///< State vector
    std::shared_ptr<biorbd::utils::Matrix> m_A; ///< Evolution matrix
    std::shared_ptr<biorbd::utils::Matrix> m_Q; ///< Noise matrix
    std::shared_ptr<biorbd::utils::Matrix> m_R; ///< Matrix of the noise on the measurements
    std::shared_ptr<biorbd::utils::Matrix> m_Pp; ///< Covariance matrix

};

}}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_H
