#ifndef MATLAB_S2M_MUSCLE_LENGTH_JACOBIAN_H
#define MATLAB_S2M_MUSCLE_LENGTH_JACOBIAN_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_muscleLengthJacobian( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( model->nbMuscleTotal(), nQ, mxREAL);
    double *Jac = mxGetPr(plhs[0]);

    s2mMatrix jaco(model->musclesLengthJacobian(*model, Q));
    int cmp(0);
    for (unsigned int j=0; j<jaco.cols(); ++j)
        for (unsigned int i=0; i<jaco.rows(); ++i){
            Jac[cmp] = jaco(i,j);
            ++cmp;
        }

    return;
}

#endif // MATLAB_S2M_MUSCLE_LENGTH_JACOBIAN_H
