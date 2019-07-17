#ifndef MATLAB_S2M_INVERSE_DYNAMICS_H
#define MATLAB_S2M_INVERSE_DYNAMICS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_inverseDynamics( int, mxArray *plhs[],
                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 6, "5 or 6 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot and 5th is QDDot and the optional 6th is the forces in global frame");
    bool externalForces = false;
    if (nrhs == 6)
        externalForces = true;

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
    std::vector<s2mGenCoord> QDDot = getParameterQddot(prhs, 4, nQdot);

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");
    if (QDDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDDot must have the same number of frames than Q");

    std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector> > f_tp;
    if (externalForces){
        f_tp = getForcePlate(prhs, 5);
        if (f_tp.size() != nFrame)
            mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Forces must have the same number of frames than Q");
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(nTau , nFrame, mxREAL);
    double *tau = mxGetPr(plhs[0]);
    unsigned int cmp(0);

    // Trouver la dynamique inverse a cette configuration
    for (unsigned int j=0; j<Q.size(); ++j){
        s2mTau Tau(Eigen::VectorXd::Zero (nTau));
        if (externalForces){
            // Recevoir les plates-formes
            std::vector<RigidBodyDynamics::Math::SpatialVector> f_ext = model->dispatchedForce(f_tp[j]);
            RigidBodyDynamics::InverseDynamics(*model, *(Q.begin()+j), *(QDot.begin()+j), *(QDDot.begin()+j), Tau, &f_ext);// Inverse Dynamics
        }
        else
            RigidBodyDynamics::InverseDynamics(*model, *(Q.begin()+j), *(QDot.begin()+j), *(QDDot.begin()+j), Tau);// Inverse Dynamics


        // Remplir l'output
        for (unsigned int i=0; i<nTau; i++){
            tau[cmp] = Tau(i);
            ++cmp;
        }
    }

    return;
}

#endif // MATLAB_S2M_INVERSE_DYNAMICS_H
