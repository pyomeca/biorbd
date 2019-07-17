#ifndef MATLAB_S2M_MUSCLES_POINTS_H
#define MATLAB_S2M_MUSCLES_POINTS_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_MusclesPoints( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */

    // Recevoir Q
    std::vector<s2mGenCoord> Q = getParameterQ(prhs, 2, nQ);

    // Cellules de sortie
    plhs[0] = mxCreateCellMatrix(model->nbMuscleTotal(), Q.size());
    // Utilisable que si nlhs >= 2
    std::vector<s2mString> wrap_forme; // forme du wrapping
    std::vector<s2mAttitude> wrap_RT; // orientation du wrapping
    std::vector<double> wrap_dim1; // dimension du wrapping
    std::vector<double> wrap_dim2; // dimension deux du wrapping


    // Stocker les position des origines/insertion/viapoints
    unsigned int cmp(0);
    for (unsigned int iQ=0; iQ<Q.size(); ++iQ)
        for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
            for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j){
                // Recueillir tout le chemin musculaire
                std::vector<s2mNodeMuscle> via(model->muscleGroup(i).muscle(j)->musclesPointsInGlobal(*model,*(Q.begin()+iQ),true));

                // Si le nombre de wrap est > 0, c'est qu'il n'y a pas de viapoints et il n'y a qu'UN wrap
                if (model->muscleGroup(i).muscle(j)->pathChanger().nbWraps() > 0){
                    // De quel type
                    s2mString forme(std::static_pointer_cast<s2mWrappingObject>(model->muscleGroup(0).muscle(0)->pathChanger().object(0))->forme());
                    wrap_forme.push_back(forme);

                    // Dans quelle orientation
                    s2mAttitude RT;
                    RT.setIdentity();
                    RT = std::static_pointer_cast<s2mWrappingObject>(model->muscleGroup(0).muscle(0)->pathChanger().object(0))->RT(*model,*(Q.begin()+iQ),false);
                    wrap_RT.push_back(RT);

                    // Quel est sa dimension
                    if (!forme.tolower().compare("cylinder")){
                        wrap_dim1.push_back(std::static_pointer_cast<s2mWrappingCylinder>(model->muscleGroup(0).muscle(0)->pathChanger().object(0))->rayon());
                        wrap_dim2.push_back(std::static_pointer_cast<s2mWrappingCylinder>(model->muscleGroup(0).muscle(0)->pathChanger().object(0))->length());
                    }
                }


                // transférer toutes les valeurs dans une variable temporaire
                mxArray *tp = mxCreateDoubleMatrix(3,via.size(),mxREAL);
                double *ptrTp = mxGetPr(tp);
                for (unsigned int k=0; k<via.size(); ++k){
                    ptrTp[k*3]   = (*(via.begin()+k))[0];
                    ptrTp[k*3+1] = (*(via.begin()+k))[1];
                    ptrTp[k*3+2] = (*(via.begin()+k))[2];
                }

                mxSetCell(plhs[0],cmp,tp);
                ++cmp;
            }

    if (nlhs >= 2){ // Sortir la déclaration des wrapping s'il y a lieu
        mwSize dims[3];
        unsigned int nWrap(static_cast<unsigned int>(wrap_RT.size()/Q.size()));
        dims[0] = nWrap;
        dims[1] = 4;
        dims[2] = Q.size();
        plhs[1] = mxCreateCellArray(3, dims);
        unsigned int cmpWrap(0);
        unsigned int cmpTime(0);

        for (unsigned int i=0; i<wrap_RT.size(); ++i){
            // Transcrire les formes dans un tableau matlab
            mxArray *tp_forme = mxCreateString( (*(wrap_forme.begin()+i)).c_str() );

            // Transcrire les valeurs RT dans un tableau matlab
            mxArray *tp_RT = mxCreateDoubleMatrix(4,4,mxREAL);
            double *RT = mxGetPr(tp_RT);
            for (unsigned int j=0;j<16;++j)
                RT[j] = (*(wrap_RT.begin()+i))(j);

            // Transcrire les valeurs de dimension1
            mxArray *tp_dim1 = mxCreateDoubleMatrix(1,1,mxREAL);
            double *dim1 = mxGetPr(tp_dim1);
            dim1[0] = *(wrap_dim1.begin()+i);
            // Transcrire les valeurs de dimension2
            mxArray *tp_dim2 = mxCreateDoubleMatrix(1,1,mxREAL);
            double *dim2 = mxGetPr(tp_dim2);
            dim2[0] = *(wrap_dim2.begin()+i);

            // Stocker les informations dans la variable de sortie
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+0*nWrap),tp_forme);
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+1*nWrap),tp_RT);
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+2*nWrap),tp_dim1);
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+3*nWrap),tp_dim2);

            cmpWrap++;
            if (cmpWrap>=nWrap){
                cmpWrap = 0;
                cmpTime++;
            }
        }
    }
    return;
}

#endif // MATLAB_S2M_MUSCLES_POINTS_H
