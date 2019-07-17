#ifndef MATLAB_S2M_NAME_BODY_H
#define MATLAB_S2M_NAME_BODY_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_nameBody( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 3, "2 arguments are required (+1 optional) where the 2nd is the handler on the model and optional third is the segment index");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    unsigned int idx;
    if (nrhs>2){
        idx = static_cast<unsigned int>(getInteger(prhs, 2, "index"));
        if (idx<1){
            std::ostringstream msg;
            msg << "Segment index must be 1 or higher.";
            mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
        }
        else if  (idx>model->nbBone()){
            std::ostringstream msg;
            msg << "Segment index must not be higher than number of segment.";
            mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
        }

        // Sortie du nom
        plhs[0] = mxCreateString (model->bone(idx-1).name().c_str()); // Recueillir le nom
    }
    else {
        // Sortie des noms
        plhs[0] = mxCreateCellMatrix(model->nbBone(), 1); // Stockage des noms de groupe

        // Stocker chaque valeur
        for (unsigned int i=0; i<model->nbBone(); ++i){
            mxArray *nomBone = mxCreateString (model->bone(i).name().c_str()); // Recueillir le nom
            mxSetCell(plhs[0],i,nomBone); // Mettre les noms dans la variable de sortie
        }
    }
    return;
}

#endif // MATLAB_S2M_NAME_BODY_H
