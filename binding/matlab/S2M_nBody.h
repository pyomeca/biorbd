#ifndef MATLAB_S2M_N_BODY_H
#define MATLAB_S2M_N_BODY_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_nBody( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL);
    double *nBody = mxGetPr(plhs[0]);

    // Get nombre de degrés de liberté
    *nBody = model->nbBone();

    return;
}

#endif // MATLAB_S2M_N_BODY_H
