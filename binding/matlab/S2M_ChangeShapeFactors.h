#ifndef MATLAB_S2M_CHANGE_SHAPE_FACTOR_H
#define MATLAB_S2M_CHANGE_SHAPE_FACTOR_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_ChangeShapeFactors( int, mxArray *[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model, 3rd is the shape factor for each muscle");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Recevoir les shape factors
    std::vector<double> shapeFactors = getDoubleArray(prhs, 2);
    if (shapeFactors.size() != model->nbMuscleTotal()){
        std::ostringstream msg;
        msg << "Length of vector of shape factors should match the number of muscles (" << model->nbMuscleTotal() << ")";
        mexErrMsgTxt(msg.str().c_str());
    }

    // Aller changer les valeurs
    unsigned int cmp(0);
    for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
        for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j){
             // Recueillir shape factor
            static_cast<s2mMuscleStateActualBuchanan*>(&(model->muscleGroup(i).muscle(j)->state_nonConst()))->shapeFactor(shapeFactors[cmp]);
            ++cmp;
        }

    return;
}

#endif // MATLAB_S2M_CHANGE_SHAPE_FACTOR_H
