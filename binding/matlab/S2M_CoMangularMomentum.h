#ifndef MATLAB_S2M_COM_ANGULAR_MOMENTUM_H
#define MATLAB_S2M_COM_ANGULAR_MOMENTUM_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_CoMangularMomentum( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4, "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is QDot");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);
    // Recevoir Qdot
    std::vector<s2mGenCoord> QDot = getParameterQdot(prhs, 3, nQdot);

    // S'assurer que Q et Qdot font la même dimension
    if (Q.size()!=QDot.size()){
        std::ostringstream msg;
        msg << "Q and Qdot must have the same number of frames";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    // Récupérer le moment angulaire
    std::vector<RigidBodyDynamics::Math::Vector3d> AM;
    for (unsigned int i=0; i<Q.size(); ++i)
        AM.push_back(model->angularMomentum(*(Q.begin()+i), *(QDot.begin()+i)));

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 3, AM.size(), mxREAL);
    double *am = mxGetPr(plhs[0]);

    // Remplir l'output
    unsigned int cmp(0);
    for (std::vector<RigidBodyDynamics::Math::Vector3d>::iterator AM_it=AM.begin(); AM_it!=AM.end(); ++AM_it)
        for (unsigned int i=0; i<3; ++i){
            am[cmp] = (*AM_it)[i];
            cmp++;
        }

    return;
}

#endif // MATLAB_S2M_COM_ANGULAR_MOMENTUM_H
