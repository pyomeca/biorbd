#ifndef MATLAB_S2M_LOCAL_JCS_H
#define MATLAB_S2M_LOCAL_JCS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_localJCS( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Trouver les RT
    std::vector<s2mAttitude> JSC_vec(model->localJCS());

    // Create a matrix for the return argument
    const mwSize dims[3]={4,4,mwSize(model->nbBone())};
    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *JCS = mxGetPr(plhs[0]);

    // Remplir l'output
    unsigned int cmpJCS = 0;
    for (std::vector<s2mAttitude>::iterator JSC_it=JSC_vec.begin(); JSC_it!=JSC_vec.end(); ++JSC_it)
        for (unsigned int i=0; i<4; ++i)
            for (unsigned int j=0; j<4; ++j){
                JCS[cmpJCS] = (*JSC_it)(j,i);
                ++cmpJCS;
            }

    return;
}

#endif // MATLAB_S2M_LOCAL_JCS_H
