#ifndef MATLAB_S2M_DELETE_H
#define MATLAB_S2M_DELETE_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_delete( int nlhs, mxArray *[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");

    // Destroy the C++ object
    destroyObject<s2mMusculoSkeletalModel>(prhs[1]);
    // Warn if other commands were ignored
    if (nlhs != 0 || nrhs != 2)
        mexWarnMsgTxt("Delete: Unexpected output arguments ignored.");
    return;
}

#endif // MATLAB_S2M_DELETE_H¸
