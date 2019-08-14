#ifndef S2M_MUSCLES_H
#define S2M_MUSCLES_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"

class s2mJoints;
namespace biorbd { namespace utils {
class String;
class Matrix;
class GenCoord;
class Tau;
}}

namespace biorbd { namespace muscles {

class MuscleGroup;
class StateDynamics;
class Force;
class MuscleNode;
class BIORBD_API Muscles
{
public:
    Muscles();
    virtual ~Muscles();

    void addMuscleGroup(
            const biorbd::utils::String &name,
            const biorbd::utils::String &originName,
            const biorbd::utils::String &insertionName);
    int getGroupId(const biorbd::utils::String &name) const; // Trouve l'index d'un groupe musculaire, -1 s'il n'a pas trouvé
    biorbd::muscles::MuscleGroup& muscleGroup_nonConst(unsigned int idx); //Retourne un groupe musculaire d'index i
    const biorbd::muscles::MuscleGroup& muscleGroup(unsigned int) const; //Retourne un groupe musculaire d'index i
    const biorbd::muscles::MuscleGroup& muscleGroup(const biorbd::utils::String&) const; //Retourne un groupe musculaire du nom demandé

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
            std::vector<std::vector<biorbd::muscles::MuscleNode>>& musclePointsInGlobal,
            std::vector<biorbd::utils::Matrix>& jacoPointsInGlobal); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(
            std::vector<std::vector<biorbd::muscles::MuscleNode>>& musclePointsInGlobal,
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
            const std::vector<biorbd::muscles::StateDynamics> &state,
            Eigen::VectorXd & F,
            bool updateKin = true,
            const biorbd::utils::GenCoord* Q = nullptr,
            const biorbd::utils::GenCoord* QDot = nullptr);
    biorbd::utils::Tau muscularJointTorque(
            s2mJoints& model,
            const std::vector<biorbd::muscles::StateDynamics> &state,
            bool updateKin = true,
            const biorbd::utils::GenCoord* Q = nullptr,
            const biorbd::utils::GenCoord* QDot = nullptr);
    biorbd::utils::Matrix musclesLengthJacobian(s2mJoints& m);
    biorbd::utils::Matrix musclesLengthJacobian(
            s2mJoints& m,
            const biorbd::utils::GenCoord& Q);
    std::vector<std::vector<std::shared_ptr<biorbd::muscles::Force>>> musclesForces(
            s2mJoints& m,
            const std::vector<biorbd::muscles::StateDynamics> &state,
            bool updateKin = true,
            const biorbd::utils::GenCoord* Q = nullptr,
            const biorbd::utils::GenCoord* QDot = nullptr);

    // Set and get
    unsigned int nbMuscleGroups() const; // retourne le nombre total de groupes musculaires
    unsigned int nbMuscleTotal() const; // retourne le nombre total de muscles
protected:
    std::vector<biorbd::muscles::MuscleGroup> m_mus;

};

}}

#endif // S2M_MUSCLES_H

