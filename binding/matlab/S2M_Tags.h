#ifndef MATLAB_S2M_TAGS_H
#define MATLAB_S2M_TAGS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_Tags( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 5, "3 arguments are required [+2 optional] where the 2nd is the handler on the model, 3rd is the Q and 4th is the wanted markerType to be return ('all', 'technical' or anatomical') and 5th if you want to remove axes as specified in the model file [default = true]");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);

    bool removeAxes(true);
    if (nrhs >= 5)
        removeAxes = getBool(prhs, 4);

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    unsigned int nTags(0); // Nombre de marqueurs
    std::vector<std::vector<s2mNodeBone> > Tags_tp; // récupérer les marqueurs
    if (nrhs >= 4){
        s2mString type(getString(prhs,3));
        if (!type.tolower().compare("all")){
            nTags = model->nTags();
            for (std::vector<s2mGenCoord>::iterator Q_it = Q.begin(); Q_it!=Q.end(); ++Q_it)
                 Tags_tp.push_back(model->Tags(*model,*Q_it, removeAxes));
        }
        else if (!type.tolower().compare("anatomical")){
                nTags = model->nAnatTags();
                for (std::vector<s2mGenCoord>::iterator Q_it = Q.begin(); Q_it!=Q.end(); ++Q_it)
                     Tags_tp.push_back(model->anatomicalTags(*model,*Q_it, removeAxes));
        }
        else if (!type.tolower().compare("technical")){
            nTags = model->nTechTags();
            for (std::vector<s2mGenCoord>::iterator Q_it = Q.begin(); Q_it!=Q.end(); ++Q_it)
                 Tags_tp.push_back(model->technicalTags(*model,*Q_it, removeAxes));
        }
        else {
            std::ostringstream msg;
            msg << "Wrong type for tags!";
            mexErrMsgTxt(msg.str().c_str());
        }

    }
    else {
        nTags = model->nTags();
        for (std::vector<s2mGenCoord>::iterator Q_it = Q.begin(); Q_it!=Q.end(); ++Q_it)
             Tags_tp.push_back(model->Tags(*model,*Q_it, removeAxes));
    }

    // Create a matrix for the return argument
    mwSize dims[3];
    dims[0] = 3;
    dims[1] = nTags;
    dims[2] = Tags_tp.size();

    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *Tags = mxGetPr(plhs[0]);

    // Remplir le output
    unsigned int cmp(0);
    for (std::vector<std::vector<s2mNodeBone> >::iterator Tags_it = Tags_tp.begin(); Tags_it!=Tags_tp.end(); ++Tags_it){
        std::vector<s2mNodeBone>::iterator it=(*Tags_it).begin();
        for (unsigned int i=0; (it+i)!=(*Tags_it).end(); ++i){
            Tags[cmp+0] = (*(it+i))(0);
            Tags[cmp+1] = (*(it+i))(1);
            Tags[cmp+2] = (*(it+i))(2);
            cmp += 3;
        }
    }

    return;
}

#endif // MATLAB_S2M_TAGS_H
