#ifndef MATLAB_S2M_FORWARD_DYNAMICS_H
#define MATLAB_S2M_FORWARD_DYNAMICS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_forwardDynamics( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 6, "5 or 6 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot and 5th is Tau and the optional 6th is a optional boolean (default = false) specifying the use of contacts");
    int contact(0);

    if (nrhs >= 6){
        if (mxIsDouble(prhs[5]))
            contact = getInteger(prhs,5);
        else if (mxIsLogical(prhs[5]))
            contact = getBool(prhs,5);
        else{
            std::ostringstream msg;
            msg << "Argument " << 6 << " must be of type bool.";
            mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
        }
    }

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
    unsigned int nQdot = model->nbQdot();
    unsigned int nQddot = model->nbQddot();
    unsigned int nTau = model->nbTau();
    unsigned int nRoot = model->nbRoot();

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);
    // Recevoir Qdot
    std::vector<s2mGenCoord> QDot = getParameterQdot(prhs, 3, nQdot);
    // Recevoir Tau
    std::vector<s2mTau> Tau = getParameterTau(prhs, 4, nTau, nRoot);

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));

    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");
    if (Tau.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Tau must have the same number of frames than Q");

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( nQddot, nFrame, mxREAL);
    double *qddot = mxGetPr(plhs[0]);
    unsigned int cmp(0), cmpForce(0);
    double *force = nullptr;
    if (contact){
        // Create a matrix for the return argument 2
        plhs[1] = mxCreateDoubleMatrix( model->nContacts(), nFrame, mxREAL);
        force = mxGetPr(plhs[1]); // Force sur les points de contacts
    }

    for (unsigned int j=0; j<nFrame; ++j){
        RigidBodyDynamics::UpdateKinematicsCustom(*model, &(*(Q.begin()+j)), &(*(QDot.begin()+j)), nullptr);

        // Trouver la dynamique directe a cette configuration
        s2mGenCoord QDDot(Eigen::VectorXd::Zero(nQddot));
        if (contact == 1){ // Si on a un contact
            model->getConstraints(*model).linear_solver = RigidBodyDynamics::Math::LinearSolverColPivHouseholderQR;
            RigidBodyDynamics::ForwardDynamicsConstraintsDirect(*model, *(Q.begin()+j), *(QDot.begin()+j), *(Tau.begin()+j), model->getConstraints(*model),QDDot);// Forward dynamics
        }
        else if (contact == -1){ // Si on a une impulsion
            s2mGenCoord QdotPost(static_cast<unsigned int>((*(Q.begin()+j)).size()));
            RigidBodyDynamics::ComputeConstraintImpulsesDirect(*model, *(Q.begin()+j), *(QDot.begin()+j), model->getConstraints(*model), QdotPost);

            // Calcul de la dynamique
            RigidBodyDynamics::ForwardDynamicsConstraintsDirect(*model, *(Q.begin()+j), QdotPost, *(Tau.begin()+j), model->getConstraints(*model),QDDot);// Forward dynamics
        }
        else {
            RigidBodyDynamics::ForwardDynamicsLagrangian(*model, *(Q.begin()+j), *(QDot.begin()+j), *(Tau.begin()+j), QDDot);// Forward dynamics
        }


        // Remplir l'output
        for (unsigned int i=0; i<nQddot; i++){
            qddot[cmp] = QDDot(i);
            ++cmp;
        }
        if (contact){
            for (unsigned int i=0; i<model->nContacts(); i++){
                force[cmpForce] = model->getConstraints().force(i);
                ++cmpForce;
            }
        }
    }
    return;
}

#endif // MATLAB_S2M_FORWARD_DYNAMICS_H
