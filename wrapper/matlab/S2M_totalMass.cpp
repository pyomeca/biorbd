void S2M_totalMass( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){
	
	// Verifier les arguments d'entrée
	// Verifier les arguments d'entrée
	checkNombreInputParametres(nrhs, 2, 2, "2 arguments are required where the 2nd is the handler on the model");
	s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
					  
	/* Create a matrix for the return argument */ 
	plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL); 
	double *mass = mxGetPr(plhs[0]);
        
	// Get la masse de tout le corps
	*mass = model->mass();
	
	return;
}