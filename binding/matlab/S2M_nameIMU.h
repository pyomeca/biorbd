#ifndef MATLAB_S2M_NAME_IMU_H
#define MATLAB_S2M_NAME_IMU_H

#include <mex.h>
#include "s2mMusculoSkeletalModel.h"
#include "class_handle.h"
#include "processArguments.h"

void S2M_nameIMU( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<s2mString> allIMU(model->IMUsNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allIMU.size(), 1);
    for (unsigned int i_bone=0; i_bone<allIMU.size(); ++i_bone){
        mxArray * imu_out_tp = mxCreateString((*(allIMU.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,imu_out_tp);
    }

    return;
}

void S2M_nameTechnicalIMU( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<s2mString> allIMU(model->technicalIMUsNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allIMU.size(), 1);
    for (unsigned int i_bone=0; i_bone<allIMU.size(); ++i_bone){
        mxArray * imu_out_tp = mxCreateString((*(allIMU.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,imu_out_tp);
    }

    return;
}

void S2M_nameAnatomicalIMU( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<s2mString> allIMU(model->anatomicalIMUsNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allIMU.size(), 1);
    for (unsigned int i_bone=0; i_bone<allIMU.size(); ++i_bone){
        mxArray * imu_out_tp = mxCreateString((*(allIMU.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,imu_out_tp);
    }

    return;
}

#endif // MATLAB_S2M_NAME_IMU_H
