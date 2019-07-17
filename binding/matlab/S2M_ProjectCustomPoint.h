#ifndef MATLAB_S2M_PROJECT_CUSTOM_POINT_H
#define MATLAB_S2M_PROJECT_CUSTOM_POINT_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_ProjectCustomPoint( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 6, 6, "6 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th are the 3xNxT points in global reference frame, 5th is the body idx to project on and 6th are axes to remove on this body axes");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Qall = getParameterQ(prhs, 2, nQ);

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    std::vector<std::vector<Eigen::Vector3d> > markersOverTime = getParameterAllMarkers(prhs,3);

    // Body index
    int bodyIdx(getInteger(prhs,4)-1);

    // Nom des axes à retirer
    s2mString axesToRemove(getString(prhs,5));

    unsigned int nFrames(static_cast<unsigned int>(markersOverTime.size()));
    if (Qall.size()!=nFrames)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Q must have the same number of frames than markers");
    int nMarker(static_cast<int>((*(markersOverTime.begin())).size()));

    // Create a matrix for the return argument
    mwSize dims[3];
    dims[0] = 3;
    dims[1] = static_cast<mwSize>(nMarker);
    dims[2] = nFrames;
    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *Markers = mxGetPr(plhs[0]);

    // Projeter les points
    unsigned int cmp(0);
    for (unsigned int i=0; i<nFrames; ++i){
        s2mGenCoord Q(*(Qall.begin()+i));
        for (unsigned int j=0; j<static_cast<unsigned int>(nMarker); ++j){
            s2mNodeBone m(*((*(markersOverTime.begin()+i)).begin()+j)  );
            s2mNodeBone tp;
            if (j==1)
                tp = model->projectPoint(Q, m.position(), bodyIdx, axesToRemove, true);
            else
                tp = model->projectPoint(Q, m.position(), bodyIdx, axesToRemove, false);
            Markers[cmp+0] = tp(0);
            Markers[cmp+1] = tp(1);
            Markers[cmp+2] = tp(2);
            cmp += 3;
        }
    }

    return;
}

#endif // MATLAB_S2M_PROJECT_CUSTOM_POINT_H
