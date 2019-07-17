#ifndef MATLAB_S2M_IMU_JACOBIAN_H
#define MATLAB_S2M_IMU_JACOBIAN_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_IMUJacobian( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */ /**** ATTENTION, NQ A REMPLACÉ NDDL, SEGFAULT? ****/

    // Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();


    // Trouver la matrice jacobienne de tous les marqueurs
    std::vector<s2mMatrix> Jac_tp = model->IMUJacobian(*model, Q);
        std::vector<s2mMatrix>::iterator it=Jac_tp.begin();

    // Create a matrix for the return argument
    unsigned int nIMUs = model->nIMUs();
    plhs[0] = mxCreateDoubleMatrix( 9*nIMUs, nQ, mxREAL);
        double *Jac = mxGetPr(plhs[0]);
    int cmp(0);
    for (unsigned int i=0; i<nQ; ++i)
        for (unsigned int j=0; (it+j)!=Jac_tp.end(); ++j)
            for (unsigned int k=0; k<9; ++k)
            {
                Jac[cmp] = (*(it+j))(k,i);
                ++cmp;
            }

    return;
}

#endif // MATLAB_S2M_IMU_JACOBIAN_H
