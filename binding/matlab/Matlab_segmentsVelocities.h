#ifndef BIORBD_MATLAB_SEGMENT_VELOCITIES_H
#define BIORBD_MATLAB_SEGMENT_VELOCITIES_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Utils/Vector.h"

void Matlab_segmentsVelocities( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4,
                               "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is QDot");

    // Recevoir le model
    biorbd::BIORBD_MATH_NAMESPACE::Model * model = convertMat2Ptr<biorbd::BIORBD_MATH_NAMESPACE::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF

    // Recevoir Q
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::GeneralizedCoordinates Q = *getParameterQ(prhs, 2,
            nQ).begin();
    // Recevoir Qdot
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::GeneralizedVelocity QDot = *getParameterQdot(prhs, 3,
            nQdot).begin();

    // Update sur la cinématique
    model->UpdateKinematicsCustom(&Q, &QDot, nullptr);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 6, nQ+1, mxREAL);
    double *vel = mxGetPr(plhs[0]);

    // Remplir l'output
    unsigned int cmp = 0;
    for (unsigned int i=0; i<model->RigidBodyDynamics::Model::v.size(); ++i){
        auto& v = model->RigidBodyDynamics::Model::v[i];
        for (unsigned int j = 0; j<6; ++j) {
            vel[i+6*cmp] = v(j);
        }
        ++cmp;
    }

    return;
}

#endif // BIORBD_MATLAB_SEGMENT_VELOCITIES_H
