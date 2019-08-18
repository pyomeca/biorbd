#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_H
#define BIORBD_RIGIDBODY_KALMAN_RECONS_H

#include <vector>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Matrix.h"

namespace biorbd {
class Model;

namespace rigidbody {
class GeneralizedCoordinates;

class BIORBD_API KalmanRecons
{
public:
    class KalmanParam{
    public:
        KalmanParam(
                double frequency = 100,
                double noiseFactor = 1e-10,
                double errorFactor = 1e-5);
        double acquisitionFrequency() const;
        double noiseFactor() const;
        double errorFactor() const;

    private:
            double m_acquisitionFrequency;
            double m_noiseFactor;
            double m_errorFactor;
    };

    // Constructeur
    KalmanRecons(
            biorbd::Model& model,
            unsigned int nMeasure,
            KalmanParam = KalmanParam());
    virtual ~KalmanRecons();

    // Recueillir l'état (Q, Qdot, Qddot)
    void getState(
            biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr);
    void setInitState(
            const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qdot = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates *Qddot = nullptr);

    // Cette fonction doit être réimplémentée de la façon nécessaire (avec les arguments souhaités). Je mets celle-ci pour force l'utilisateur à implémenter au moins cette fonction.
    // Ceci dit, sans argument (particulièrement les données d'entrées), il est douteux que la réimplémentation puisse fonctionner
    virtual void reconstructFrame() = 0;

protected:
    // Calculs internes
    virtual void initialize();
    biorbd::utils::Matrix evolutionMatrix(
            const unsigned int m,
            unsigned int n,
            double Te); // Création de la matrice d'évolution
    biorbd::utils::Matrix processNoiseMatrix(
            const unsigned int nQ,
            double Te);
    biorbd::utils::Matrix measurementNoiseMatrix(
            const unsigned int nT,
            double MN);
    biorbd::utils::Matrix initCovariance(
            const unsigned int nQ,
            double csnt);
    biorbd::rigidbody::GeneralizedCoordinates initState(const unsigned int nQ);
    void iteration(
            Eigen::VectorXd measure,
            const Eigen::VectorXd &projectedMeasure,
            const biorbd::utils::Matrix &Hessian,
            const std::vector<unsigned int> &occlusion = std::vector<unsigned int>());
    virtual void manageOcclusionDuringIteration(
            biorbd::utils::Matrix &InvTp,
            Eigen::VectorXd &measure,
            const std::vector<unsigned int> &occlusion);

    // Attributs variables
    KalmanParam m_params; // Fréquence d'acquisition
    double m_Te; // Parametre inérant a la frequence
    unsigned int m_nDof; // Nombre de DoF, calculé des qu'un modele est ouvert
    unsigned int m_nMeasure; // Nombre de marqueurs

    // Attributs du filtre de kalman
    Eigen::VectorXd m_xp; // Vecteur d'état
    biorbd::utils::Matrix m_A; // Matrice d'évolution
    biorbd::utils::Matrix m_Q; // Matrice de bruit
    biorbd::utils::Matrix m_R; // Matrice de bruit de la mesure
    biorbd::utils::Matrix m_Pp; // Matrice de covariance

};

}}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_H
