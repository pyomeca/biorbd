#ifndef MATLAB_BIORBD_MUSCLES_FORCE_H
#define MATLAB_BIORBD_MUSCLES_FORCE_H

#include <mex.h>
#include "BiorbdModel.h"
#include "Muscles/Force.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_MusclesForce( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 6, "5 arguments are required [+1 optional] where the 2nd is the handler on the model, 3rd is the Q, 4th is Qdot, 5th is muscles states and optional 6th is if update [true] must be done. Note that if update is set to [false], the user MUST update it by himself before calling this function");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs, 2, nQ);

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedVelocity> Qdot = getParameterQdot(prhs, 3, nQdot);

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (Qdot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");

    // Recevoir les états musculaires
    std::vector<std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>>> state = getParameterMuscleStateActivation(prhs, 4, model->nbMuscleTotal());

    bool updateKin(true);
    if (nrhs >= 6){ // Si on doit récupérer si on update ou pas
        updateKin = getBool(prhs, 5);
        if (!updateKin && nFrame > 1) // Si on n'update pas, s'assurer qu'un seul frame est envoyé
            mexErrMsgIdAndTxt("MATLAB:dim:WrongDimension", "No update is incompatible with more than one frame.");
    }

    // Cellules de sortie
    mwSize dims[4];
    dims[0] = 3;
    dims[1] = 2;
    dims[2] = model->nbMuscleTotal();
    dims[3] = nFrame;
    plhs[0] = mxCreateNumericArray(4, dims, mxDOUBLE_CLASS, mxREAL);
    double *muscleForce = mxGetPr(plhs[0]);

    // Aller chercher les valeurs
    unsigned int cmp(0);
    for (unsigned int iF=0; iF<nFrame; ++iF){
        std::vector<std::vector<std::shared_ptr<biorbd::muscles::Force>>> Force;
        if (updateKin)
            Force = model->musclesForces(state[iF], updateKin, &Q[iF], &Qdot[iF]);
        else
            Force = model->musclesForces(state[iF], updateKin);
        for (unsigned int i=0; i<Force.size(); ++i){
            const biorbd::muscles::Force& ori = *(Force[i][0]);
            const biorbd::muscles::Force& ins = *(Force[i][1]);
            muscleForce[6*cmp+0] = ori[0];
            muscleForce[6*cmp+1] = ori[1];
            muscleForce[6*cmp+2] = ori[2];
            muscleForce[6*cmp+3] = ins[0];
            muscleForce[6*cmp+4] = ins[1];
            muscleForce[6*cmp+5] = ins[2];
            ++cmp;
        }
    }

    return;
}

#endif // MATLAB_BIORBD_MUSCLES_FORCE_H
