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
    ~s2mMuscles();

    virtual void addMuscleGroup(const s2mString &name, const s2mString &originName, const s2mString &insertionName);
    virtual s2mGroupeMusculaire& muscleGroup(const unsigned int); //Retourne un groupe musculaire d'index i
    virtual s2mGroupeMusculaire& muscleGroup(const s2mString&); //Retourne un groupe musculaire du nom demandé
    virtual void updateMuscles(s2mJoints&, const s2mGenCoord& Q, bool); // Update les positions/jacobiennes/vitesse, etc
    virtual void updateMuscles(s2mJoints&, const s2mGenCoord& Q, const s2mGenCoord& QDot, bool); // Update les positions/jacobiennes/vitesse, etc
    virtual void updateMuscles(std::vector<std::vector<s2mNodeMuscle> >& musclePointsInGlobal, std::vector<Eigen::MatrixXd>& jacoPointsInGlobal); // Update les positions/jacobiennes/vitesse, etc
    virtual void updateMuscles(std::vector<std::vector<s2mNodeMuscle> >& musclePointsInGlobal, std::vector<Eigen::MatrixXd>& jacoPointsInGlobal, const s2mGenCoord& QDot); // Update les positions/jacobiennes/vitesse, etc
    int getGroupId(const s2mString &name); // Trouve l'index d'un groupe musculaire, -1 s'il n'a pas trouvé

    // Calcul des effets musculaires sur les os
    s2mTau muscularJointTorque(s2mJoints& model, const Eigen::VectorXd & F, bool updateKin = true, const s2mGenCoord* Q = NULL, const s2mGenCoord* QDot = NULL);
    s2mTau muscularJointTorque(s2mJoints& model, const std::vector<s2mMuscleStateActual> &state, Eigen::VectorXd & F, bool updateKin = true, const s2mGenCoord* Q = NULL, const s2mGenCoord* QDot = NULL);
    s2mTau muscularJointTorque(s2mJoints& model, const std::vector<s2mMuscleStateActual> &state, bool updateKin = true, const s2mGenCoord* Q = NULL, const s2mGenCoord* QDot = NULL);
    Eigen::MatrixXd musclesLengthJacobian(s2mJoints& m, bool updateKinLevel = true, const s2mGenCoord* Q = NULL);
    std::vector<std::vector<boost::shared_ptr<s2mMuscleForce> > > musclesForces(s2mJoints& m, const std::vector<s2mMuscleStateActual> &state, bool updateKin = true, const s2mGenCoord* Q = NULL, const s2mGenCoord* QDot = NULL);

    // Set and get
    virtual unsigned int nbMuscleGroups(); // retourne le nombre total de groupes musculaires
    unsigned int nbMuscleTotal(); // retourne le nombre total de muscles
protected:
    std::vector<s2mGroupeMusculaire> m_mus;
private:
};

#endif // S2MMUSCLES_H

