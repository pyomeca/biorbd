#ifndef BIORBD_MATLAB_NAME_TAGS_H
#define BIORBD_MATLAB_NAME_TAGS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_nameTags( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allTags(model->markerNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allTags.size(), 1);
    for (unsigned int i_bone=0; i_bone<allTags.size(); ++i_bone){
        mxArray * tags_out_tp = mxCreateString((*(allTags.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,tags_out_tp);
    }

    return;
}

void Matlab_nameTechnicalTags( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allTags(model->technicalMarkerNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allTags.size(), 1);
    for (unsigned int i_bone=0; i_bone<allTags.size(); ++i_bone){
        mxArray * tags_out_tp = mxCreateString((*(allTags.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,tags_out_tp);
    }

    return;
}

void Matlab_nameAnatomicalTags( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

     // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allTags(model->anatomicalMarkerNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allTags.size(), 1);
    for (unsigned int i_bone=0; i_bone<allTags.size(); ++i_bone){
        mxArray * tags_out_tp = mxCreateString((*(allTags.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,tags_out_tp);
    }

    return;
}

#endif // BIORBD_MATLAB_NAME_TAGS_H
