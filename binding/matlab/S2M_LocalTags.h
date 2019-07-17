#ifndef MATLAB_S2M_LOCAL_TAGS_H
#define MATLAB_S2M_LOCAL_TAGS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_LocalTags( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 4, "2 arguments are required [+2 optional] where the 2nd is the handler on the model, 3rd is the wanted markerType to be return ('all' [default], 'technical' or anatomical') and 4th if you want to remove axes as specified in the model file [default = true]");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Gestion du type
    s2mString type("all");
    if (nrhs >= 3)
        type = getString(prhs,2);
    bool removeAxes(true);
    if (nrhs >= 4)
        removeAxes = getBool(prhs, 3);

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    unsigned int nTags(0); // Nombre de marqueurs
    std::vector<s2mNodeBone> Tags_tp; // récupérer les marqueurs
    if (!type.tolower().compare("all")){
            nTags = model->nTags();
            Tags_tp = model->s2mMarkers::Tags(removeAxes);
        }
        else if (!type.tolower().compare("anatomical")){
            nTags = model->nAnatTags();
            Tags_tp = model->s2mMarkers::anatomicalTags(removeAxes);
        }
        else if (!type.tolower().compare("technical")){
            nTags = model->nTechTags();
            Tags_tp = model->s2mMarkers::technicalTags(removeAxes);
        }
        else {
            std::ostringstream msg;
            msg << "Wrong type for tags!";
            mexErrMsgTxt(msg.str().c_str());
        }

    // Create a matrix for the return argument
    mwSize dims[2];
    dims[0] = 3;
    dims[1] = nTags;

    plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
    double *Tags = mxGetPr(plhs[0]);

    // Remplir le output
    unsigned int cmp(0);
    std::vector<s2mNodeBone>::iterator it=Tags_tp.begin();
        for (unsigned int i=0; (it+i)!=Tags_tp.end(); ++i){
            Tags[cmp+0] = (*(it+i))(0);
            Tags[cmp+1] = (*(it+i))(1);
            Tags[cmp+2] = (*(it+i))(2);
            cmp += 3;
        }

    return;
}

#endif // MATLAB_S2M_LOCAL_TAGS_H
