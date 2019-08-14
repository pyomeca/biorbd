#ifndef MATLAB_S2M_CONTACT_JACOBIAN_H
#define MATLAB_S2M_CONTACT_JACOBIAN_H

#include <mex.h>
#include <rbdl/Constraints.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_ContactJacobian( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    biorbd::utils::GenCoord Q = *getParameterQ(prhs, 2, nQ).begin();

    // Trouver la matrice jacobienne de tous les contacts
    unsigned int nContacts = model->nContacts();
    Eigen::MatrixXd G_tp(Eigen::MatrixXd::Zero(nContacts,model->nbQ()));
    RigidBodyDynamics::CalcConstraintsJacobian(*model, Q, model->getConstraints_nonConst(*model), G_tp, true);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( nContacts, nQ, mxREAL);
    double *Jac = mxGetPr(plhs[0]);
    for (unsigned int j=0; j<nContacts; ++j)
        for (unsigned int i=0; i<nQ; ++i)
            Jac[j+i*nContacts] = G_tp(j,i);

    return;
}

#endif // MATLAB_S2M_CONTACT_JACOBIAN_H
