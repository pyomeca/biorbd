#ifndef MATLAB_S2M_CONTACT_GAMMA_H
#define MATLAB_S2M_CONTACT_GAMMA_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_ContactGamma( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4, "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is the Qdot");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */ /**** ATTENTION, NQ A REMPLACÉ NDDL, SEGFAULT? ****/
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */ /**** ATTENTION, NQ A REMPLACÉ NDDL, SEGFAULT? ****/

    // Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();
    s2mGenCoord QDot = *getParameterQdot(prhs, 3, nQdot).begin();
    unsigned int nContacts = model->nContacts();

    Eigen::MatrixXd G_tp(Eigen::MatrixXd::Zero(nContacts,model->nbQ()));
    RigidBodyDynamics::CalcConstraintsJacobian(*model, Q, model->getConstraints(*model), G_tp, true);

    RigidBodyDynamics::Math::VectorNd Gamma = model->getConstraints(*model).gamma;

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( nContacts, 1, mxREAL);
    double *gamma = mxGetPr(plhs[0]);
    for (unsigned int j=0; j<nContacts; ++j)
        gamma[j] = Gamma[j];

    return;
}

#endif // MATLAB_S2M_CONTACT_GAMMA_H
