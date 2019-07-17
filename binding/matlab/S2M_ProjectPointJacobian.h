#ifndef MATLAB_S2M_PROJECT_POINT_JACOBIAN_H
#define MATLAB_S2M_PROJECT_POINT_JACOBIAN_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_ProjectPointJacobian( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q and 4th are the 3xN markers where N=nTags of the model");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    std::vector<Eigen::Vector3d> markersOverTime = *getParameterAllMarkers(prhs,3).begin();

    // Trouver la matrice jacobienne de tous les marqueurs
    std::vector<s2mMatrix> Jac_tp = model->projectPointJacobian(*model, *model, Q, markersOverTime, true);
        std::vector<s2mMatrix>::iterator it=Jac_tp.begin();

    // Create a matrix for the return argument
    unsigned int nTags = model->nTags();
    plhs[0] = mxCreateDoubleMatrix( 3*nTags, nQ, mxREAL);
        double *Jac = mxGetPr(plhs[0]);
     for (unsigned int j=0; (it+j)!=Jac_tp.end(); ++j)
        for (unsigned int i=0; i<nQ; ++i)
            for (unsigned int k=0; k<3; ++k)
                Jac[j+nTags*k+i*nTags*3] = (*(it+j))(k,i);

    return;
}

#endif // MATLAB_S2M_PROJECT_POINT_JACOBIAN_H
