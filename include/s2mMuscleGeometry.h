#ifndef S2MMUSCLEGEOMETRY_H
#define S2MMUSCLEGEOMETRY_H

    #include <Eigen/Dense>
    #include "biorbdConfig.h"
    #include "s2mNodeMuscle.h"
    #include "s2mJoints.h"
    #include "s2mMusclePathChangers.h"
    #include "s2mGenCoord.h"
    #include "s2mMuscleCaracteristics.h"

class BIORBD_API s2mMuscleGeometry
{
    public:
        s2mMuscleGeometry(const s2mNodeMuscle &origin = s2mNodeMuscle(), const s2mNodeMuscle &insertion = s2mNodeMuscle());
        ~s2mMuscleGeometry();

        // Fonction a appeler avant d'appeler longueur/velocity ou autres!
        void updateKinematics(s2mJoints &model,
                              const s2mGenCoord* Q = NULL,
                              const s2mGenCoord* Qdot = NULL,
                              const s2mMuscleCaracteristics& = s2mMuscleCaracteristics(),
                              const s2mMusclePathChangers& = s2mMusclePathChangers(),
                              int updateKin = 2);
        void updateKinematics(std::vector<s2mNodeMuscle>& musclePointsInGlobal,
                              Eigen::MatrixXd& jacoPointsInGlobal,
                              const s2mGenCoord* Qdot = NULL,
                              const s2mMuscleCaracteristics& = s2mMuscleCaracteristics());

        // Get and set des position d'origine et insertions
        s2mNodeMuscle originInLocal() const;
        s2mNodeMuscle insertionInLocal() const;
        void originInLocal(const s2mNodeMuscle &val);
        void insertionInLocal(const s2mNodeMuscle &val);

        // Position des muscles dans l'espace
        s2mNodeMuscle originInGlobal() const; // Attention il est impératif d'avoir update une premiere fois pour que ceci fonctionne!
        s2mNodeMuscle insertionInGlobal() const; // Attention il est impératif d'avoir update une premiere fois pour que ceci fonctionne!
        std::vector<s2mNodeMuscle> musclesPointsInGlobal() const; // Return already computed via points

        // Retour des longueur et vitesse musculaires
        double length() const; // Return the already computed muscle length
        double musculoTendonLength() const; // Return the already computed muscle-tendon length
        double velocity() const; // Return the already computed velocity

        // Retour des jacobiennes
        Eigen::MatrixXd jacobian() const; // Retourne la derniere jacobienne
        Eigen::MatrixXd jacobianOrigin() const;
        Eigen::MatrixXd jacobianInsertion() const ;
        Eigen::MatrixXd jacobian(const unsigned int i) const;
        Eigen::MatrixXd jacobianLength() const;


    protected:
        // Update commun de la cinématique
        void _updateKinematics(const s2mGenCoord *Qdot, const s2mMuscleCaracteristics &c, const s2mMusclePathChangers* o = NULL);

        // Calcul de la position des points dans le global
        s2mNodeMuscle originInGlobal(s2mJoints &model, const s2mGenCoord &Q); // Update la cinématique puis retourne la position de l'origine dans l'espace
        s2mNodeMuscle insertionInGlobal(s2mJoints &model, const s2mGenCoord &Q); // Update la cinématique puis retourne la position de l'insertion dans l'espace
        void musclesPointsInGlobal(std::vector<s2mNodeMuscle>& ptsInGlobal); // Forcer les points dans le global
        void musclesPointsInGlobal(s2mJoints &, const s2mGenCoord &, const s2mMusclePathChangers&);

        // Calcul de la longueur musculaire
        double length(const s2mMuscleCaracteristics&, const s2mMusclePathChangers* pathChanger = NULL); // Update the kinematics and compute and return muscle length
        // Calcul de la vitesse musculaire
        double velocity(const s2mGenCoord &Qdot); // Update the kinematics and compute and return muscle velocity assuming no via points nor wrapping objects
        // Calcul des jacobiennes des points
        void jacobian(Eigen::MatrixXd& jaco); // Forcer une jacobienne
        void jacobian(s2mJoints &model, const s2mGenCoord &Q);
        void computeJacobianLength();

        // Position des nodes dans le repere local
        s2mNodeMuscle m_origin; // Node de l'origine
        s2mNodeMuscle m_insertion; // Node de l'insertion

        s2mNodeMuscle m_originInGlobal; // Position de l'origine dans le repere global
        s2mNodeMuscle m_insertionInGlobal; // Position de l'origine dans le repere global
        std::vector<s2mNodeMuscle> m_pointsInGlobal; // position de tous les points dans le global
        std::vector<s2mNodeMuscle> m_pointsInLocal; // position de tous les points dans le global
        Eigen::MatrixXd m_jacobian;
        Eigen::MatrixXd m_jacobianLength; // Incluant la

        double m_length; // Longueur du muscle
        double m_muscleTendonLength; // Longueur du muscle et du tendon
        double m_velocity; // Vitesse de l'élongation musculaire

        bool m_isGeometryComputed; // Savoir si la geometry a été faite au moins une fois
        bool m_isVelocityComputed; // Savoir si dans le dernier update, la vitesse a été calculée
        bool m_posAndJacoWereForced; // On a utilisé la procédure d'override de la position du muscle et de la jacobienne
    private:
};

#endif // S2MMUSCLEGEOMETRY_H
