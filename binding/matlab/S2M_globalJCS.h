#ifndef MATLAB_S2M_GLOBAL_JCS_H
#define MATLAB_S2M_GLOBAL_JCS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_globalJCS( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);

    // Trouver les RT
    std::vector<std::vector<s2mAttitude> > JSC_vec;
    for (std::vector<s2mGenCoord>::iterator Q_it = Q.begin(); Q_it!=Q.end(); ++Q_it)
        JSC_vec.push_back(model->globalJCS(*Q_it));

    // Create a matrix for the return argument
    const mwSize dims[4]={4,4,mwSize(model->nbBone()),mwSize(JSC_vec.size())};
    plhs[0] = mxCreateNumericArray(4, dims, mxDOUBLE_CLASS, mxREAL);
    double *JCS = mxGetPr(plhs[0]);

    // Remplir l'output
    unsigned int cmpJCS = 0;
    for (std::vector<std::vector<s2mAttitude> >::iterator AllJCS_it = JSC_vec.begin(); AllJCS_it != JSC_vec.end(); ++AllJCS_it)
        for (std::vector<s2mAttitude>::iterator JSC_it=(*AllJCS_it).begin(); JSC_it!=(*AllJCS_it).end(); ++JSC_it)
            for (unsigned int i=0; i<4; ++i)
                for (unsigned int j=0; j<4; ++j){
                    JCS[cmpJCS] = (*JSC_it)(j,i);
                    ++cmpJCS;
                }

    return;
}

#endif // MATLAB_S2M_GLOBAL_JCS_H
