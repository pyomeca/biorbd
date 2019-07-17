#ifndef MATLAB_S2M_MUSCLES_FORCE_MAX_H
#define MATLAB_S2M_MUSCLES_FORCE_MAX_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_MusclesForceMax( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 3, "2 arguments are required (+1 optional) where the 2nd is the handler on the model. If 2 arguments were sent, the function returns values for each muscles, if a 3rd is sent, the max forces are replace by the sent values.");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    double *muscleForceMax = nullptr; // Pointeur pour la sortie
    Eigen::VectorXd Forces;
    if (nrhs == 2){
        // Cellules de sortie
        mwSize dims[2];
        dims[0] = model->nbMuscleTotal();
        dims[1] = 1;
        plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
        muscleForceMax = mxGetPr(plhs[0]);
    }
    else{
        Forces = getVector(prhs, 2, model->nbMuscleTotal(), "MuscleForce");
    }

    // Aller chercher les valeurs
    unsigned int cmp(0);
    for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
        for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j){
            if (nrhs == 2) // Recueillir toutes les forces max
                muscleForceMax[cmp]  = model->muscleGroup(i).muscle(j)->caract().forceIsoMax();
            else // Remplacer les valeurs
                model->muscleGroup(i).muscle(j)->forceIsoMax(Forces(cmp));
            ++cmp;
        }

    return;
}

#endif // MATLAB_S2M_MUSCLES_FORCE_MAX_H
