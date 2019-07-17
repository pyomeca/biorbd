#ifndef MATLAB_S2M_COM_H
#define MATLAB_S2M_COM_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_CoM( int nlhs, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);

    // Create a matrix for the return argument
    mwSize dims[3];
    dims[0] = 3;
    dims[1] = 1;
    dims[2] = Q.size();
    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *com = mxGetPr(plhs[0]);

    // Trouver la position du CoM
    for (unsigned int i=0; i<Q.size(); ++i){
        RigidBodyDynamics::Math::Vector3d COM = model->CoM(*(Q.begin()+i));
        for (unsigned int j=0; j<3; ++j)
            com[3*i+j] = COM(j);
    }

    return;
}

#endif // MATLAB_S2M_COM_H
