#ifndef MATLAB_BIORBD_H
#define MATLAB_BIORBD_H

#include <mex.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iostream>

#include "s2mMusculoSkeletalModel.h"
#include "s2mRead.h"
#include "class_handle.h"
#include "s2mKalmanReconsMarkers.h"
#include "s2mKalmanReconsIMU.h"
#include "s2mMuscleOptimisation.h"
#include "s2mMatrix.h"

#include "S2M_help.h"
#include "S2M_new.h"
#include "S2M_delete.h"
#include "S2M_changeGravity.h"
#include "S2M_totalMass.h"
#include "S2M_massMatrix.h"
#include "S2M_nControl.h"
#include "S2M_nQ.h"
#include "S2M_nQdot.h"
#include "S2M_nQddot.h"
#include "S2M_nameDof.h"
#include "S2M_nRoot.h"
#include "S2M_nTags.h"
#include "S2M_nameTags.h"
#include "S2M_nBody.h"
#include "S2M_nameBody.h"
#include "S2M_segmentMass.h"
#include "S2M_Tags.h"
#include "S2M_ProjectCustomPoint.h"
#include "S2M_ProjectPoint.h"
#include "S2M_ProjectPointJacobian.h"
#include "S2M_nIMU.h"
#include "S2M_nameIMU.h"
#include "S2M_IMU.h"
#include "S2M_IMUJacobian.h"
#include "S2M_LocalTags.h"
#include "S2M_localJCS.h"
#include "S2M_segmentTags.h"
#include "S2M_inverseKinematics.h"
#include "S2M_inverseKinematicsEKF.h"
#include "S2M_inverseKinematicsEKF_IMU.h"
#include "S2M_Mesh.h"
#include "S2M_Patch.h"
#include "S2M_TagsJacobian.h"
#include "S2M_ContactsPosition.h"
#include "S2M_ContactJacobian.h"
#include "S2M_ContactGamma.h"
#include "S2M_CoM.h"
#include "S2M_CoMJacobian.h"
#include "S2M_CoMdot.h"
#include "S2M_CoMddot.h"
#include "S2M_segmentCoM.h"
#include "S2M_segmentCoMdot.h"
#include "S2M_segmentCoMddot.h"
#include "S2M_CoMangularMomentum.h"
#include "S2M_segmentAngularMomentum.h"
#include "S2M_segmentsInertia.h"
#include "S2M_segmentsVelocities.h"
#include "S2M_globalJCS.h"
#include "S2M_muscleUpdate.h"
#include "S2M_MusclesPoints.h"
#include "S2M_MusclesJacobian.h"
#include "S2M_MusclesLength.h"
#include "S2M_muscleLengthJacobian.h"
#include "S2M_muscleVelocity.h"
#include "S2M_MusclesNames.h"
#include "S2M_MusclesParentNames.h"
#include "S2M_nMuscles.h"
#include "S2M_muscleJointTorqueFromActivation.h"
#include "S2M_muscleJointTorqueFromExcitation.h"
#include "S2M_muscleJointTorqueFromMuscleForce.h"
#include "S2M_MusclesForce.h"
#include "S2M_MusclesForceMax.h"
#include "S2M_muscleForcesNorm.h"
#include "S2M_MusclesActivationDot.h"
#include "S2M_MusclesExcitationDotBuchanan.h"
#include "S2M_ChangeShapeFactors.h"
#include "S2M_MaximeMuscleOptim.h"
#include "S2M_parent.h"

#include "S2M_inverseDynamics.h"
#include "S2M_torqueActivation.h"
#include "S2M_NLeffects.h"
#include "S2M_forwardDynamics.h"
#include "S2M_computeQdot.h"

#ifdef _WIN32
// This is a hack because Eigen can't be dynamically compiled on Windows, while dlib needs consistency in compilation. 
// Please note that this can result in undefined behavior while using s2mMuscleOptimisation...
const int USER_ERROR__inconsistent_build_configuration__see_dlib_faq_1_ = 0;
const int DLIB_VERSION_MISMATCH_CHECK__EXPECTED_VERSION_19_10_0 = 0;
#endif

std::string toLower(const std::string &str){
    std::string new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::tolower);
    return new_str;
}

// MATLAB INTERFACE 
/** \brief Entry point for the muscod application */
void mexFunction( int nlhs, mxArray *plhs[], 
                  int nrhs, const mxArray*prhs[] );
void functionHub( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){
    /* Check for proper number of arguments */
    if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:yprime:invalidNumInputs",
                "First argument should be the command");
    }

    // Traitement du 1er argument qui est la commande
    /* Check for proper input type */
    if (!mxIsChar(prhs[0]) || (mxGetM(prhs[0]) != 1 ) )  {
        mexErrMsgIdAndTxt( "MATLAB:mxmalloc:invalidInput",
                "Input argument 1 must be a command string.");
    }
    char *cmd_char = mxArrayToString(prhs[0]);
    std::string cmd(cmd_char);
    mxFree(cmd_char);

    // Si on a demandé de l'aide
    if (!toLower(cmd).compare("help")){//!strcmp("help", cmd)){
        S2M_help();
        return;
    }


    // Redirect vers les bonnes fonctions

    /* À l'appel d'un nouveau modèle */
    if (!toLower(cmd).compare("new")){
        S2M_new(nlhs, plhs, nrhs, prhs);
        return;
    }

    /* À l'appel du delete du modèle */
    if (!toLower(cmd).compare("delete")){
        S2M_delete(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Changer la gravité du modèle
    if (!toLower(cmd).compare("gravity")){
        S2M_changeGravity(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de controls (tau)
    if (!toLower(cmd).compare("ncontrol")){
        s2mError::s2mWarning(0, "La fonction \"nControl\" est obsolete. Remplacer par \"nTau\". Elle sera retirée prochainement");
        S2M_nTau(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de Tau
    if (!toLower(cmd).compare("ntau")){
        S2M_nTau(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de dof sur le segment racine
    if (!toLower(cmd).compare("nroot")){
        S2M_nRoot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de dof sur le segment racine
    if (!toLower(cmd).compare("parent")){
        S2M_parent(nlhs, plhs, nrhs, prhs);
        return;
    }


    // mass totale du systeme
    if (!toLower(cmd).compare("totalmass")){
        S2M_totalMass(nlhs, plhs, nrhs, prhs);
        return;
    }

    // matrice de masse en fonction de la position
    if(!toLower(cmd).compare("massmatrix")){
        S2M_massMatrix(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de dof
    if (!toLower(cmd).compare("nq")){
        S2M_nQ(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de qdot
    if (!toLower(cmd).compare("nqdot")){
        S2M_nQdot(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Nombre de dof
    if (!toLower(cmd).compare("nqddot")){
        S2M_nQddot(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Nombre de dof
    if (!toLower(cmd).compare("namedof")){
        S2M_nameDof(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de Tags
    if (!toLower(cmd).compare("ntags")){
        S2M_nTags(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Tags
    if (!toLower(cmd).compare("nametags")){
        S2M_nameTags(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Noms des Tags techniques
    if (!toLower(cmd).compare("nametechnicaltags")){
        S2M_nameTechnicalTags(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Noms des Tags anatomiques
    if (!toLower(cmd).compare("nameanatomicaltags")){
        S2M_nameAnatomicalTags(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Local Tags
    if (!toLower(cmd).compare("localtags")){
        S2M_LocalTags(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de segments
    if (!toLower(cmd).compare("nbody")){
        S2M_nBody(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Noms des segments
    if (!toLower(cmd).compare("namebody")){
        S2M_nameBody(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Masse des segments
    if (!toLower(cmd).compare("segmentmass")){
        S2M_segmentMass(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("tags")){
        S2M_Tags(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("projectpoint")){
        S2M_ProjectPoint(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Fonction de cinématique directe
    if(!toLower(cmd).compare("projectpointjacobian")){
        S2M_ProjectPointJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("projectcustompoint")){
        S2M_ProjectCustomPoint(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de Tags
    if (!toLower(cmd).compare("nmimu")){
        s2mError::s2mWarning(0, "La fonction \"nmimu\" est obsolete. Remplacer par \"nimu\". Elle sera retirée prochainement");
        S2M_nIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nimu")){
        S2M_nIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Tags
    if (!toLower(cmd).compare("namemimu")){
        s2mError::s2mWarning(0, "La fonction \"namemimu\" est obsolete. Remplacer par \"nameimu\". Elle sera retirée prochainement");
        S2M_nameIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nameimu")){
        S2M_nameIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Tags techniques
    if (!toLower(cmd).compare("nametechnicalmimu")){
        s2mError::s2mWarning(0, "La fonction \"nametechnicalmimu\" est obsolete. Remplacer par \"nametechnicalimu\". Elle sera retirée prochainement");
        S2M_nameTechnicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nametechnicalimu")){
        S2M_nameTechnicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Tags anatomiques
    if (!toLower(cmd).compare("nameanatomicalmimu")){
        s2mError::s2mWarning(0, "La fonction \"nameanatomicalmimu\" est obsolete. Remplacer par \"nameanatomicalimu\". Elle sera retirée prochainement");
        S2M_nameAnatomicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nameanatomicalmimu")){
        S2M_nameAnatomicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("mimu")){
        s2mError::s2mWarning(0, "La fonction \"mimu\" est obsolete. Remplacer par \"imu\". Elle sera retirée prochainement");
        S2M_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("imu")){
        S2M_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("mimujacobian")){
        s2mError::s2mWarning(0, "La fonction \"mimujacobian\" est obsolete. Remplacer par \"imujacobian\". Elle sera retirée prochainement");
        S2M_IMUJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("imujacobian")){
        S2M_IMUJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Fonction de cinématique inverse
    if(!toLower(cmd).compare("ik")){
        S2M_inverseKinematics(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique inverse
    if(!toLower(cmd).compare("ik_ekf")){
        S2M_inverseKinematicsEKFallInOneCall(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique inverse par filtre de kalman
    if(!toLower(cmd).compare("ik_ekf_new")){
        S2M_setEKF(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_step")){
        S2M_inverseKinematicsEKFstep(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_delete")){
        S2M_delEKF(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique inverse
    if(!toLower(cmd).compare("ik_ekf_imu")){
        S2M_inverseKinematicsEKF_IMUallInOneCall(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique inverse par filtre de kalman
    if(!toLower(cmd).compare("ik_ekf_imu_new")){
        S2M_setEKF_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_imu_step")){
        S2M_inverseKinematicsEKF_IMUstep(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_imu_delete")){
        S2M_delEKF_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("segmentstags")){
        S2M_segmentTags(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de la Jacobienne de la cinématique directe
    if(!toLower(cmd).compare("tagsjacobian")){
        S2M_TagsJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe pour les points de contacts
    if(!toLower(cmd).compare("contacts")){
        S2M_ContactsPosition(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("contactjacobian")){
        S2M_ContactJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("contactgamma")){
        S2M_ContactGamma(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver le Centre de masse
    if(!toLower(cmd).compare("com")){
        S2M_CoM(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour obtenir la jacobienne du centre de masse
    if(!toLower(cmd).compare("comjacobian")){
        S2M_CoMJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver la vitesse du Centre de masse
    if(!toLower(cmd).compare("comdot")){
        S2M_CoMdot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'acceleration du Centre de masse
    if(!toLower(cmd).compare("comddot")){
        S2M_CoMddot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'acceleration du Centre de masse
    if(!toLower(cmd).compare("mesh")){
        S2M_Mesh(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'acceleration du Centre de masse
    if(!toLower(cmd).compare("patch")){
        S2M_Patch(nlhs, plhs, nrhs, prhs);
        return;
    }

    // moment angulaire du centre de masse
    if(!toLower(cmd).compare("comangularmomentum")){
        S2M_CoMangularMomentum(nlhs, plhs, nrhs, prhs);
        return;
    }

    // moment angulaire du centre de masse
    if(!toLower(cmd).compare("segmentangularmomentum")){
        S2M_segmentAngularMomentum(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver le Centre de masse des segments
    if(!toLower(cmd).compare("segmentcom")){
        S2M_segmentCOM(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver la vitesse Centre de masse des segments
    if(!toLower(cmd).compare("segmentcomdot")){
        S2M_segmentCOMdot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'accélération Centre de masse des segments
    if(!toLower(cmd).compare("segmentcomddot")){
        S2M_segmentCOMddot(nlhs, plhs, nrhs, prhs);
        return;
    }


    if(!toLower(cmd).compare("segmentsvelocities")){
        S2M_segmentsVelocities(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Inerties segmentaires
    if(!toLower(cmd).compare("segmentsinertia")){
        S2M_segmentsInertia(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("segmentsinertialocal")){
        S2M_segmentsInertiaLocal(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Obtenir les JCS dans le global
    if(!toLower(cmd).compare("globaljcs")){
        S2M_globalJCS(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir les JCS dans le global
    if(!toLower(cmd).compare("localjcs")){
        S2M_localJCS(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Obtenir le nombre de muscles
    if(!toLower(cmd).compare("nmuscles")){
        S2M_nMuscles(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir le noms des muscles
    if(!toLower(cmd).compare("musclesnames")){
        S2M_MusclesNames(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir le noms des muscles
    if(!toLower(cmd).compare("musclesparentnames")){
        S2M_MusclesParentNames(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir la position des muscles dans le global
    if(!toLower(cmd).compare("musclepoints")){
        S2M_MusclesPoints(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("muscleupdate")){
        S2M_muscleUpdate(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir la position des muscles dans le global
    if(!toLower(cmd).compare("musclelength")){
        S2M_MusclesLength(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir la position des muscles dans le global
    if(!toLower(cmd).compare("muscletendonlength")){
        S2M_MusclesTendonLength(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Obtenir la vitesse d'élongation des muscles
    if(!toLower(cmd).compare("musclevelocity")){
        S2M_muscleVelocity(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de dynamique inverse
    if(!toLower(cmd).compare("inversedynamics")){
        S2M_inverseDynamics(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Si on veut convertir des activations de torque en torque
    if(!toLower(cmd).compare("torqueactivation")){
        S2M_torqueActivation(nlhs, plhs, nrhs, prhs);
        return;
    }


    if(!toLower(cmd).compare("forwarddynamics")){
        S2M_forwardDynamics(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("computeqdot")){
        S2M_computeQdot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de dynamique inverse
    if(!toLower(cmd).compare("nleffects")){
        S2M_NLeffects(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("jointtorquefromforce")){
        S2M_muscleJointTorqueFromMuscleForce(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("jointtorquefromactivation")){
        S2M_muscleJointTorqueFromActivation(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("jointtorquefromexcitation")){
        S2M_muscleJointTorqueFromExcitation(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("muscleactivationdot")){
        S2M_MusclesActivationDot(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("muscleexcitationdotbuchanan")){
        S2M_MusclesExcitationDotBuchanan(nlhs, plhs, nrhs, prhs);
        return;
    }


    if(!toLower(cmd).compare("muscleforce")){
        S2M_MusclesForce(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("muscleforcemax")){
        S2M_MusclesForceMax(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("muscleforcenorm")){
        S2M_muscleForcesNorm(nlhs, plhs, nrhs, prhs);
        return;
    }


    if(!toLower(cmd).compare("musclejacobian")){
        S2M_MusclesJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("musclelengthjacobian")){
        S2M_muscleLengthJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

#ifdef S2M_MUSCLE_OPTIMIZATION
    if(!toLower(cmd).compare("maximeoptim")){
        S2M_MaximeMuscleOptim(nlhs, plhs, nrhs, prhs);
        return;
    }
#endif

    if(!toLower(cmd).compare("changeshapefactors")){
        S2M_ChangeShapeFactors(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Got here, so command is not recognized
    mexErrMsgTxt("Command for interface not recognized.");
}

#endif // MATLAB_BIORBD_H
