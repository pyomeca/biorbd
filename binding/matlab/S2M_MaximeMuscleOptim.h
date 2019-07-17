#ifndef MATLAB_S2M_MAXIME_MUSCLE_OPTIM_H
#define MATLAB_S2M_MAXIME_MUSCLE_OPTIM_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"
#ifdef S2M_MUSCLE_OPTIMIZATION
void S2M_MaximeMuscleOptim( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 9, 9, "9 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot, 5th is QDDot, 6th is Tau, 7th are muscle states, 8th is dof of Arm Flexion and 9th is initial guess (or previous answer)");
    bool externalForces = false;
    if (nrhs == 6)
        externalForces = true;

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */
    unsigned int nQddot = model->nbQddot(); /* Get the number of DoF */
    unsigned int nTau = model->nbTau(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);
    // Recevoir Qdot
    std::vector<s2mGenCoord> QDot = getParameterQdot(prhs, 3, nQdot);
    // Recevoir Qddot
    std::vector<s2mGenCoord> QDDot = getParameterQddot(prhs, 4, nQddot);
    // Recevoir Tau
    std::vector<s2mGenCoord> Tau = getParameterQ(prhs, 5, nTau, "Tau");

    // Recevoir muscleStates
    std::vector<std::vector<s2mMuscleStateActual> > s = getParameterMuscleStateActivation(prhs,6,model->nbMuscleTotal());
    // Recevoir le dof a optimiser
    unsigned int dofFlex = static_cast<unsigned int>(getInteger(prhs,7));
    // Recevoir la valeur initiale
    double x_init = getDouble(prhs,8);

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");
    if (QDDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDDot must have the same number of frames than Q");
    if (Tau.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Tau must have the same number of frames than Q");
    if (s.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "s must have the same number of frames than Q");

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(1 , 1, mxREAL);

    // Faire l'optimisation du gain
    s2mMuscleOptimisation::OptimData d(*model,Q,QDot,QDDot,Tau,s,dofFlex);
    std::vector<s2mMuscleOptimisation::OptimData> d_all;
    d_all.push_back(d);
    s2mMuscleOptimisation::parameter_vector x;
    x(0) = x_init;
    s2mMuscleOptimisation::optimizeJointTorque(d_all, x);

    // Retourner la valeur
    double *out = mxGetPr(plhs[0]);
    out[0] = x(0);

        return;
}
#endif // S2M_MUSCLE_OPTIMIZATION
#endif // MATLAB_S2M_MAXIME_MUSCLE_OPTIM_H
