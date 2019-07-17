#ifndef MATLAB_S2M_SEGMENT_MASS_H
#define MATLAB_S2M_SEGMENT_MASS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_segmentMass( int, mxArray *plhs[],
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

        // Sortie des masses
        plhs[0] = mxCreateDoubleMatrix (1,1,mxREAL);
        double *mass = mxGetPr(plhs[0]);
        mass[0] = model->bone(idx-1).caract().mass(); // Mettre les masses dans la variable de sortie
    }
    else {
        // Sortie des noms
        plhs[0] = mxCreateDoubleMatrix(model->nbBone(), 1, mxREAL); // Stockage des noms de groupe
        double *mass = mxGetPr(plhs[0]);

        // Stocker chaque valeur
        for (unsigned int i=0; i<model->nbBone(); ++i)
            mass[i] = model->bone(i).caract().mass(); // Mettre les masses dans la variable de sortie
    }

    return;
}

#endif // MATLAB_S2M_SEGMENT_MASS_H
