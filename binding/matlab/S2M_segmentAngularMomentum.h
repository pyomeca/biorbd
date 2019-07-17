#ifndef MATLAB_S2M_SEGMENT_ANGULAR_MOMENTUM_H
#define MATLAB_S2M_SEGMENT_ANGULAR_MOMENTUM_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_segmentAngularMomentum( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 5, "4 arguments are required (+1 optional) where the 2nd is the handler on the model, 3rd is the Q, 4th is the Qdot and 5th is the index of body segment");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */ /***** ATTENTION NDDL A ÉTÉ REMPLACÉ PAR NQ, SEGFAULT? ***/
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);
    // Recevoir QDot
    std::vector<s2mGenCoord> QDot = getParameterQdot(prhs, 3, nQdot);

    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");

    // Recevoir l'index du segment
    int idx(-1);
    if (nrhs==5){
        idx = getInteger(prhs, 4) - 1; // -1 ainsi le premier segment est 1
        if (idx >= static_cast<int>(nQ) || idx < 0)
            mexErrMsgIdAndTxt("MATLAB:model:invalidSegmentInput",
                              "Selected segment doesn't exist");
    }

    // Create a matrix for the return argument
    if (idx == -1){
        mwSize dims[3];
        dims[0] = 3;
        dims[1] = nQ;
        dims[2] = nFrame;
        plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    }
    else{
        mwSize dims[3];
        dims[0] = 3;
        dims[1] = 1;
        dims[2] = nFrame;
        plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    }
    double *angularMomentum = mxGetPr(plhs[0]);


    // Préparer la sortie conditionnelle (tous si idx==-1, seulement celui demandé sinon)
    unsigned int cmp(0);
    for (unsigned int i=0; i<nFrame; ++i){
        std::vector<RigidBodyDynamics::Math::Vector3d> am_all(model->CalcSegmentsAngularMomentum (*model, *(Q.begin()+i), *(QDot.begin()+i), true));
        if (idx==-1){
            // Remplir le output
            std::vector<RigidBodyDynamics::Math::Vector3d>::iterator it=am_all.begin();
            for (unsigned int i=0; (it+i)!=am_all.end(); ++i){
                angularMomentum[cmp*3  ] = (*(it+i))(0);
                angularMomentum[cmp*3+1] = (*(it+i))(1);
                angularMomentum[cmp*3+2] = (*(it+i))(2);
                cmp++;
            }
        }
        else{
            // Remplir le output
            std::vector<RigidBodyDynamics::Math::Vector3d>::iterator it=am_all.begin();
            angularMomentum[i*3+0] = (*(it+idx))(0);
            angularMomentum[i*3+1] = (*(it+idx))(1);
            angularMomentum[i*3+2] = (*(it+idx))(2);
        }
    }

    return;
}

#endif // MATLAB_S2M_SEGMENT_ANGULAR_MOMENTUM_H
