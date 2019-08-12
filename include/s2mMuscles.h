#ifndef S2M_MUSCLES_H
#define S2M_MUSCLES_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"

class s2mJoints;
class s2mGroupeMusculaire;
class s2mNodeMuscle;
namespace biorbd { namespace utils {
class String;
class Matrix;
class GenCoord;
class Tau;
}}
class s2mMuscleStateDynamics;
class s2mMuscleForce;
class BIORBD_API s2mMuscles
{
public:
    s2mMuscles();
    virtual ~s2mMuscles();

    void addMuscleGroup(
            const biorbd::utils::String &name,
            const biorbd::utils::String &originName,
            const biorbd::utils::String &insertionName);
    int getGroupId(const biorbd::utils::String &name) const; // Trouve l'index d'un groupe musculaire, -1 s'il n'a pas trouvé
    s2mGroupeMusculaire& muscleGroup_nonConst(unsigned int idx); //Retourne un groupe musculaire d'index i
    const s2mGroupeMusculaire& muscleGroup(unsigned int) const; //Retourne un groupe musculaire d'index i
    const s2mGroupeMusculaire& muscleGroup(const biorbd::utils::String&) const; //Retourne un groupe musculaire du nom demandé

    void updateMuscles(
            s2mJoints&,
            const biorbd::utils::GenCoord& Q,
            bool); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(
            s2mJoints&,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord& QDot,
            bool); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(
            std::vector<std::vector<s2mNodeMuscle>>& musclePointsInGlobal,
            std::vector<biorbd::utils::Matrix>& jacoPointsInGlobal); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(
            std::vector<std::vector<s2mNodeMuscle>>& musclePointsInGlobal,
            std::vector<biorbd::utils::Matrix>& jacoPointsInGlobal,
            const biorbd::utils::GenCoord& QDot); // Update les positions/jacobiennes/vitesse, etc

    // Calcul des effets musculaires sur les os
    biorbd::utils::Tau muscularJointTorque(
            s2mJoints& model,
            const Eigen::VectorXd & F,
            bool updateKin = true,
            const biorbd::utils::GenCoord* Q = nullptr,
            const biorbd::utils::GenCoord* QDot = nullptr);
    biorbd::utils::Tau muscularJointTorque(
            s2mJoints& model,
            const std::vector<s2mMuscleStateDynamics> &state,
            Eigen::VectorXd & F,
            bool updateKin = true,
            const biorbd::utils::GenCoord* Q = nullptr,
            const biorbd::utils::GenCoord* QDot = nullptr);
    biorbd::utils::Tau muscularJointTorque(
            s2mJoints& model,
            const std::vector<s2mMuscleStateDynamics> &state,
            bool updateKin = true,
            const biorbd::utils::GenCoord* Q = nullptr,
            const biorbd::utils::GenCoord* QDot = nullptr);
    biorbd::utils::Matrix musclesLengthJacobian(s2mJoints& m);
    biorbd::utils::Matrix musclesLengthJacobian(
            s2mJoints& m,
            const biorbd::utils::GenCoord& Q);
    std::vector<std::vector<std::shared_ptr<s2mMuscleForce>>> musclesForces(
            s2mJoints& m,
            const std::vector<s2mMuscleStateDynamics> &state,
            bool updateKin = true,
            const biorbd::utils::GenCoord* Q = nullptr,
            const biorbd::utils::GenCoord* QDot = nullptr);

    // Set and get
    unsigned int nbMuscleGroups() const; // retourne le nombre total de groupes musculaires
    unsigned int nbMuscleTotal() const; // retourne le nombre total de muscles
protected:
    std::vector<s2mGroupeMusculaire> m_mus;

};

#endif // S2M_MUSCLES_H

