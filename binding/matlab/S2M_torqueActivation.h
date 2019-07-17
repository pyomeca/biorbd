#ifndef MATLAB_S2M_TORQUE_ACTIVATION_H
#define MATLAB_S2M_TORQUE_ACTIVATION_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_torqueActivation( int, mxArray *plhs[],
                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 5, "5 or 5 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot and 5th is Tau");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */
    unsigned int nTau = model->nbTau() + model->nbRoot(); /* Nombre de Tau */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);
    // Recevoir Qdot
    std::vector<s2mGenCoord> QDot = getParameterQdot(prhs, 3, nQdot);
    // Recevoir Qddot
    std::vector<s2mTau> act = getParameterTau(prhs, 4, model->nbTau(), model->nbRoot());

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");
    if (act.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Tau must have the same number of frames than Q");

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(model->nbTau() , nFrame, mxREAL);
    double *tau = mxGetPr(plhs[0]);
    unsigned int cmp(0);

    // Trouver la dynamique inverse a cette configuration
    for (unsigned int j=0; j<Q.size(); ++j){
        // Calcul des couples
        Eigen::VectorXd Tau = model->torque(*model, (*(act.begin()+j)), (*(Q.begin()+j)), (*(QDot.begin()+j)));

        // Remplir l'output
        for (unsigned int i=model->nbRoot(); i<nTau; i++){
            tau[cmp] = Tau(i);
            ++cmp;
        }
    }

    return;
}

#endif // MATLAB_S2M_TORQUE_ACTIVATION_H
