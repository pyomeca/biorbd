#ifndef MATLAB_S2M_NAME_DOF_H
#define MATLAB_S2M_NAME_DOF_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_nameDof( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Sortie des noms
    plhs[0] = mxCreateCellMatrix(nQ, 1); // Stockage des noms de groupe

    // Stocker chaque valeur
    unsigned int cmp(0);
    for (unsigned int i=0; i<model->nbBone(); ++i){
        for (unsigned int j=0; j<model->bone(i).nDof(); ++j){
            mxArray *nomDof = mxCreateString ((model->bone(i).name() + "_" + model->bone(i).nameDof(j)).c_str()); // Recueillir le nom
            mxSetCell(plhs[0],cmp,nomDof); // Mettre les noms dans la variable de sortie
            ++cmp;
        }
    }

    return;
}

#endif // MATLAB_S2M_NAME_DOF_H
