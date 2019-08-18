#ifndef BIORBD_MUSCLES_MUSCLES_H
#define BIORBD_MUSCLES_MUSCLES_H

#include <vector>
#include <memory>

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Matrix;
}

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
}

namespace muscles {
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
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& QDot,
            bool); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(
            std::vector<std::vector<biorbd::muscles::MuscleNode>>& musclePointsInGlobal,
            std::vector<biorbd::utils::Matrix>& jacoPointsInGlobal); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(
            std::vector<std::vector<biorbd::muscles::MuscleNode>>& musclePointsInGlobal,
            std::vector<biorbd::utils::Matrix>& jacoPointsInGlobal,
            const biorbd::rigidbody::GeneralizedCoordinates& QDot); // Update les positions/jacobiennes/vitesse, etc

    // Calcul des effets musculaires sur les os
    biorbd::rigidbody::GeneralizedTorque muscularJointTorque(
            const Eigen::VectorXd & F,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);
    biorbd::rigidbody::GeneralizedTorque muscularJointTorque(
            const std::vector<biorbd::muscles::StateDynamics> &state,
            Eigen::VectorXd & F,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);
    biorbd::rigidbody::GeneralizedTorque muscularJointTorque(
            const std::vector<biorbd::muscles::StateDynamics> &state,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);
    biorbd::utils::Matrix musclesLengthJacobian();
    biorbd::utils::Matrix musclesLengthJacobian(const biorbd::rigidbody::GeneralizedCoordinates& Q);
    std::vector<std::vector<std::shared_ptr<biorbd::muscles::Force>>> musclesForces(
            const std::vector<biorbd::muscles::StateDynamics> &state,
            bool updateKin = true,
            const biorbd::rigidbody::GeneralizedCoordinates* Q = nullptr,
            const biorbd::rigidbody::GeneralizedCoordinates* QDot = nullptr);

    // Set and get
    unsigned int nbMuscleGroups() const; // retourne le nombre total de groupes musculaires
    unsigned int nbMuscleTotal() const; // retourne le nombre total de muscles
protected:
    std::vector<biorbd::muscles::MuscleGroup> m_mus;

};

}}

#endif // BIORBD_MUSCLES_MUSCLES_H

