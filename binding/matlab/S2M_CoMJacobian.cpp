#ifndef MATLAB_S2M_COM_JACOBIAN_H
#define MATLAB_S2M_COM_JACOBIAN_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_CoMJacobian( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int NDDL = model->nbDof(); /* Get the number of DoF */
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();

    // Trouver la jacobienne du COM
    RigidBodyDynamics::Math::MatrixNd Jaco = model->CoMJacobian(Q).transpose();

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 3, nQ, mxREAL);
    double *jaco = mxGetPr(plhs[0]);

    // Mettre les valeurs dans la sortie;
    for (unsigned int i=0; i<nQ*3; ++i)
                jaco[i] = Jaco(i);

    return;
}

#endif // MATLAB_S2M_COM_JACOBIAN_H
