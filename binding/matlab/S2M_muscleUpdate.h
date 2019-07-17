#ifndef MATLAB_S2M_MUSCLE_UPDATE_H
#define MATLAB_S2M_MUSCLE_UPDATE_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_muscleUpdate( int, mxArray *[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    //checkNombreInputParametres(nrhs, 6, 7, "6 arguments are required [+1 optional] where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot, 5th is all muscles points (origin, via points, insertion), 6th is the muscle point Jacobian and optional 7th is a 1xN matrix depicting the number of point for each muscle (default is the one given in the model)");
    checkNombreInputParametres(nrhs, 6, 6, "6 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot, 5th is all muscles points (origin, via points, insertion), 6th is the muscle point Jacobian");

    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
    unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);
    // Recevoir Qdot
    std::vector<s2mGenCoord> QDot = getParameterQdot(prhs, 3, nQdot);

    // S'assurer qu'il n'y a qu'un seul frame
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (nFrame != 1)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "Q must be exactly composed of 1 frame");
    if (QDot.size() != nFrame)
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension", "QDot must have the same number of frames than Q");


    // Si on n'envoie pas le nombre de via par muscles, assumer que c'est eux du modèle
    Eigen::VectorXd nPoints;
//    if (nrhs >= 7){
//        nPoints = getVector(prhs, 6, "ViaPoints");
//    } else {
        // Compter le nombre d'élément à mettre dans nPoints
        int nMus(0);
        for (unsigned int i = 0; i<model->nbMuscleGroups(); ++i)
            nMus += model->muscleGroup(i).nbMuscles();
        nPoints = Eigen::VectorXd(nMus);

        // Mettre le nombre nécessaire dans chaque case
        int cmpMus(0);
        for (unsigned int i = 0; i<model->nbMuscleGroups(); ++i){
            s2mGroupeMusculaire grMus(model->muscleGroup(i));
            for (unsigned int j = 0; j<grMus.nbMuscles(); ++j){
                nPoints(cmpMus) = grMus.muscle(j)->pathChanger().nbObjects() + 2; // nombre d'objet + origine + insertion
                ++cmpMus;
            }
        }
//    }

    // Recueillir la matrice de points
    std::vector<std::vector<s2mNodeMuscle> > musclePosition(getMusclePosition(prhs, 4, nPoints));

    // Recueillir la matrice jacobienne
    std::vector<s2mMatrix> musclePointsJaco(getMusclePointsJaco(prhs, 5, nPoints, nQ));

    // Appeler la fonction d'update
    model->updateMuscles(musclePosition, musclePointsJaco, *(QDot.begin()) ); // Update les positions/jacobiennes/vitesse, etc

    return;
}

#endif // MATLAB_S2M_MUSCLE_UPDATE_H
