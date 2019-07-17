#ifndef MATLAB_S2M_MUSCLE_JOINT_TORQUE_FROM_MUSCLE_FORCE_H
#define MATLAB_S2M_MUSCLE_JOINT_TORQUE_FROM_MUSCLE_FORCE_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_muscleJointTorqueFromMuscleForce( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 6, "5 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot, 5th is the muscles force norm vector and optional 6th is if update [true] must be done. Note that if update is set to [false], the user MUST update it by himself before calling this function");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */
    unsigned int nTau = model->nbTau(); /* Get the number of DoF */
    unsigned int nRoot = model->nbRoot(); /* Get the number of DoF */
    unsigned int nMuscleTotal = model->nbMuscleTotal();

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);
    // Recevoir Qdot
    std::vector<s2mGenCoord> QDot = getParameterQdot(prhs, 3, nQdot);
    // Recevoir muscleStates
    std::vector<Eigen::VectorXd> Fm = getParameterMuscleForceNorm(prhs,4,nMuscleTotal);

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");
    if (Fm.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Muscles force must have the same number of frames than Q");

    bool updateKin(true);
    if (nrhs >= 6){ // Si on doit récupérer si on update ou pas
        updateKin = getBool(prhs, 5);
        if (!updateKin && nFrame > 1) // Si on n'update pas, s'assurer qu'un seul frame est envoyé
            mexErrMsgIdAndTxt("MATLAB:dim:WrongDimension", "No update is incompatible with more than one frame.");
    }

    // Create a matrix for the return argument
    mwSize dims[2];
    dims[0] = nTau;
    dims[1] = nFrame;
    plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
    double *Tau = mxGetPr(plhs[0]);

    // Remplir le output
    for (unsigned int i=0; i<nFrame; ++i){
        s2mTau muscleTorque;
        if (updateKin)
            muscleTorque = model->muscularJointTorque(*model, *(Fm.begin()+i), updateKin, &(*(Q.begin()+i)), &(*(QDot.begin()+i)));
        else
            muscleTorque = model->muscularJointTorque(*model, *(Fm.begin()+i), updateKin);

        // distribuer les Tau
        for (unsigned int j=0; j<nTau; ++j){
            Tau[i*nTau+j] = muscleTorque(j+nRoot);
        }

    }

    return;
}

#endif // MATLAB_S2M_MUSCLE_JOINT_TORQUE_FROM_MUSCLE_FORCE_H
