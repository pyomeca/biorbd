 
void S2M_nMuscles( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);

    // Sortie des noms
    plhs[0] =  mxCreateDoubleMatrix( 1, 1, mxREAL);
    double *nMus = mxGetPr(plhs[0]);

    // Récupérer le nombre de muscles
    *nMus = model->nbMuscleTotal();

    return;
}
