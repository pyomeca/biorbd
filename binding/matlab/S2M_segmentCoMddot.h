#ifndef MATLAB_S2M_SEGMENT_COM_DDOT_H
#define MATLAB_S2M_SEGMENT_COM_DDOT_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_segmentCOMddot( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] ){
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 6, "5 arguments are required (+1 optional) where the 2nd is the handler on the model, 3rd is the Q, 4th is the Qdot, 5th is Qddot and 6th is the index of body segment");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */
    unsigned int nQddot = model->nbQddot(); /* Get the number of DoF */

    // Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();
    // Recevoir Qdot
    s2mGenCoord QDot = *getParameterQdot(prhs, 3, nQdot).begin();
    // Recevoir Qddot
    s2mGenCoord QDDot = *getParameterQddot(prhs, 4, nQddot).begin();
    // Recevoir le numéro du segment (optionnel)
    int i(0);
    if (nrhs==6)
        i = getInteger(prhs, 5);
    i -= 1; // -1 car le segment 0 est numéroté 1 sous matlab

    // Trouver la vitesse du CoM
    if (i==-1){
        std::vector<RigidBodyDynamics::Math::Vector3d> COMddot = model->CoMddotBySegment(Q,QDot,QDDot,true);
        // Create a matrix for the return argument
        plhs[0] = mxCreateDoubleMatrix( 3, model->nbBone(), mxREAL);
        // Remplir l'output
        double *tp = mxGetPr(plhs[0]);
        for (unsigned int j=0; j<model->nbBone(); ++j)
                for (unsigned int k=0; k<3; ++k)
                        tp[3*j+k] = COMddot[j][k]; // Transférer le tout dans un tableau de sortie
    }
    else {
        RigidBodyDynamics::Math::Vector3d COMddot = model->CoMddotBySegment(Q,QDot,QDDot,static_cast<unsigned int>(i),true);

        /* Create a matrix for the return argument */
        plhs[0] = mxCreateDoubleMatrix( 3, 1, mxREAL);
        // Remplir l'output
        double *tp = mxGetPr(plhs[0]);
        for (unsigned int k=0; k<3; ++k)
                tp[k] = COMddot[k]; // Transférer le tout dans un tableau de sortie
    }

    return;
}

#endif // MATLAB_S2M_SEGMENT_COM_DDOT_H
