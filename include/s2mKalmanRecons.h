#ifndef KALMANRECONS_HPP
#define KALMANRECONS_HPP

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "s2mString.h"
#include "s2mGenCoord.h"
#include "s2mMusculoSkeletalModel.h"

class BIORBD_API s2mKalmanRecons
{
public:
    class s2mKalmanParam{
        public:
        s2mKalmanParam(double frequency = 100, double noiseFactor = 1e-10, double errorFactor = 1e-5):
            m_acquisitionFrequency(frequency),
            m_noiseFactor(noiseFactor),
            m_errorFactor(errorFactor){}
            double acquisitionFrequency(){return m_acquisitionFrequency;}
            double noiseFactor(){return m_noiseFactor;}
            double errorFactor(){return m_errorFactor;}

        private:
            double m_acquisitionFrequency;
            double m_noiseFactor;
            double m_errorFactor;
    };

    // Constructeur
    s2mKalmanRecons(s2mMusculoSkeletalModel&, unsigned int nMeasure, s2mKalmanParam = s2mKalmanParam());
    virtual ~s2mKalmanRecons();

    // Recueillir l'état (Q, Qdot, Qddot)
    void getState(s2mGenCoord *Q = nullptr, s2mGenCoord *Qdot = nullptr, s2mGenCoord *Qddot = nullptr);
    void setInitState(const s2mGenCoord *Q = nullptr, const s2mGenCoord *Qdot = nullptr, const s2mGenCoord *Qddot = nullptr);

    // Cette fonction doit être réimplémentée de la façon nécessaire (avec les arguments souhaités). Je mets celle-ci pour force l'utilisateur à implémenter au moins cette fonction.
    // Ceci dit, sans argument (particulièrement les données d'entrées), il est douteux que la réimplémentation puisse fonctionner
    virtual void reconstructFrame() = 0;

protected:
    // Calculs internes
    virtual void initialize();
    Eigen::MatrixXd evolutionMatrix(const unsigned int m, unsigned int n, const double Te); // Création de la matrice d'évolution
    Eigen::MatrixXd processNoiseMatrix(const unsigned int nQ, const double Te);
    Eigen::MatrixXd measurementNoiseMatrix(const unsigned int nT, const double MN);
    Eigen::MatrixXd initCovariance(const unsigned int nQ, const double csnt);
    s2mGenCoord initState(const unsigned int nQ);
    void iteration(Eigen::VectorXd measure, const Eigen::VectorXd &projectedMeasure, const Eigen::MatrixXd &Hessian, const std::vector<unsigned int> &occlusion = std::vector<unsigned int>());
    virtual void manageOcclusionDuringIteration(Eigen::MatrixXd &InvTp, Eigen::VectorXd &measure, const std::vector<unsigned int> &occlusion);

    // Attributs variables
    s2mKalmanParam m_params; // Fréquence d'acquisition
    double m_Te; // Parametre inérant a la frequence
    unsigned int m_nDof; // Nombre de DoF, calculé des qu'un modele est ouvert
    unsigned int m_nMeasure; // Nombre de marqueurs

    // Attributs du filtre de kalman
    Eigen::VectorXd m_xp; // Vecteur d'état
    Eigen::MatrixXd m_A; // Matrice d'évolution
    Eigen::MatrixXd m_Q; // Matrice de bruit
    Eigen::MatrixXd m_R; // Matrice de bruit de la mesure
    Eigen::MatrixXd m_Pp; // Matrice de covariance

};

#endif // KALMANRECONS_HPP
