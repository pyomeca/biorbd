#ifndef MATLAB_BIORBD_MUSCLES_FORCE_NORM_H
#define MATLAB_BIORBD_MUSCLES_FORCE_NORM_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Muscles/Force.h"

void Matlab_muscleForcesNorm( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 6, "5 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is Qdot, 5th is muscles states and optional 6th is if update [true] must be done. Note that if update is set to [false], the user MUST update it by himself before calling this function");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs, 2, nQ);

    // Recevoir QDot
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> QDot = getParameterQdot(prhs, 3, nQdot);

    // Recevoir les états musculaires
    std::vector<std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>>> state = getParameterMuscleStateActivation(prhs, 4, model->nbMuscleTotal());


    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");
    if (state.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Muscles states must have the same number of frames than Q");

    bool updateKin = true;
    if (nrhs >= 6){ // Si on doit récupérer si on update ou pas
        updateKin = getBool(prhs, 5);
        if (!updateKin && nFrame > 1) // Si on n'update pas, s'assurer qu'un seul frame est envoyé
            mexErrMsgIdAndTxt("MATLAB:dim:WrongDimension", "No update is incompatible with more than one frame.");
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( model->nbMuscleTotal(), nFrame, mxREAL);
    double *force = mxGetPr(plhs[0]);

    unsigned int cmp=0;
    for (unsigned int iF=0; iF<nFrame; ++iF){
        std::vector<std::vector<std::shared_ptr<biorbd::muscles::Force>>> Force;
        if (updateKin)
            Force = model->musclesForces(state[iF], updateKin, &Q[iF], &QDot[iF]);
        else
            Force = model->musclesForces(state[iF], updateKin);

        for (unsigned int i=0; i<Force.size(); ++i){
            force[cmp] = Force[i][0]->norm();
            ++cmp;
        }
    }

    return;
}

#endif // MATLAB_BIORBD_MUSCLES_FORCE_NORM_H

