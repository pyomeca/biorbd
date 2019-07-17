#ifndef MATLAB_S2M_NEW_H
#define MATLAB_S2M_NEW_H

#include <iostream>
#include <mex.h>
#include "s2mRead.h"
#include "class_handle.h"

void S2M_new( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){
    if (nrhs != 2) {
        mexErrMsgIdAndTxt( "MATLAB:yprime:invalidNumInputs",
                "2 arguments are required where the 2nd is the path to the model");
    }

    // Get the path of the model
    if (!mxIsChar(prhs[1]) || (mxGetM(prhs[1]) != 1 ) )  {
    mexErrMsgIdAndTxt( "MATLAB:mxmalloc:invalidInput",
            "Input argument 2 must be a file path string.");
    }
    if (nlhs != 1) {
    mexErrMsgIdAndTxt( "MATLAB:yprime:maxlhs",
            "You must catch the pointer address!");
    }
    char *buf = mxArrayToString(prhs[1]);
    std::string filepath(buf); /* Copier le cstring dans un std::string */

    // Loader le modèle musculosquelettique
    // Definition des variables globales du modele
    if (s2mRead::is_readable( filepath )){
        try{
            plhs[0] = convertPtr2Mat<s2mMusculoSkeletalModel>(new s2mMusculoSkeletalModel(s2mRead::readModelFile(filepath)));
        }
        catch (std::string m){
            mexErrMsgTxt(m.c_str());
        }

        return;
    }
    else {
        std::cout << filepath << " est inexistant ou non lisible.\n";
        (void) plhs;    /* unused parameters */
        return;
    }
}
#endif // MATLAB_S2M_NEW_H
