
void S2M_changeGravity   ( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){
	
	// Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the gravity field 3d vector");
	s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
					  
	/* Create a matrix for the return argument */ 
	plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL); 

    Eigen::Vector3d grav = getVector3d(prhs, 2);

    model->gravity = grav;
}
