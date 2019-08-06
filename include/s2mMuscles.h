#ifndef S2MMUSCLES_H
#define S2MMUSCLES_H

#include "biorbdConfig.h"
#include "s2mError.h"
#include <vector>
#include "s2mGroupeMusculaire.h"
#include "s2mGenCoord.h"
#include "s2mTau.h"

class BIORBD_API s2mMuscles
{
public:
    s2mMuscles();
    virtual ~s2mMuscles();

    void addMuscleGroup(const s2mString &name, const s2mString &originName, const s2mString &insertionName);
    int getGroupId(const s2mString &name) const; // Trouve l'index d'un groupe musculaire, -1 s'il n'a pas trouvé
    s2mGroupeMusculaire& muscleGroup_nonConst(unsigned int idx); //Retourne un groupe musculaire d'index i
    const s2mGroupeMusculaire& muscleGroup(unsigned int) const; //Retourne un groupe musculaire d'index i
    const s2mGroupeMusculaire& muscleGroup(const s2mString&) const; //Retourne un groupe musculaire du nom demandé

    void updateMuscles(s2mJoints&, const s2mGenCoord& Q, bool); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(s2mJoints&, const s2mGenCoord& Q, const s2mGenCoord& QDot, bool); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(std::vector<std::vector<s2mNodeMuscle>>& musclePointsInGlobal, std::vector<s2mMatrix>& jacoPointsInGlobal); // Update les positions/jacobiennes/vitesse, etc
    void updateMuscles(std::vector<std::vector<s2mNodeMuscle>>& musclePointsInGlobal, std::vector<s2mMatrix>& jacoPointsInGlobal, const s2mGenCoord& QDot); // Update les positions/jacobiennes/vitesse, etc

    // Calcul des effets musculaires sur les os
    s2mTau muscularJointTorque(s2mJoints& model, const Eigen::VectorXd & F, bool updateKin = true, const s2mGenCoord* Q = nullptr, const s2mGenCoord* QDot = nullptr);
    s2mTau muscularJointTorque(s2mJoints& model, const std::vector<s2mMuscleStateDynamics> &state, Eigen::VectorXd & F, bool updateKin = true, const s2mGenCoord* Q = nullptr, const s2mGenCoord* QDot = nullptr);
    s2mTau muscularJointTorque(s2mJoints& model, const std::vector<s2mMuscleStateDynamics> &state, bool updateKin = true, const s2mGenCoord* Q = nullptr, const s2mGenCoord* QDot = nullptr);
    s2mMatrix musclesLengthJacobian(s2mJoints& m, const s2mGenCoord& Q = s2mGenCoord());
    std::vector<std::vector<std::shared_ptr<s2mMuscleForce>>> musclesForces(s2mJoints& m, const std::vector<s2mMuscleStateDynamics> &state, bool updateKin = true, const s2mGenCoord* Q = nullptr, const s2mGenCoord* QDot = nullptr);

    // Set and get
    unsigned int nbMuscleGroups() const; // retourne le nombre total de groupes musculaires
    unsigned int nbMuscleTotal() const; // retourne le nombre total de muscles
protected:
    std::vector<s2mGroupeMusculaire> m_mus;
private:
};

#endif // S2MMUSCLES_H

