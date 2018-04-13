

void S2M_massMatrix( int nlhs, mxArray *plhs[],
				int nrhs, const mxArray*prhs[] ){
	// Verifier les arguments d'entrée
	checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");
	
	// Recevoir le model
	s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
    unsigned int nQ = model->nbQ(); /* Get the number of DoF */
	
	// Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();
		
	// Trouver la matrice de masse
    RigidBodyDynamics::Math::MatrixNd Mass(nQ,nQ);
    Mass.setZero();
	RigidBodyDynamics::UpdateKinematicsCustom(*model, &Q, NULL, NULL);
	RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model, Q, Mass, false);
	
	/* Create a matrix for the return argument */ 
    plhs[0] = mxCreateDoubleMatrix( nQ, nQ, mxREAL);
	double *mass = mxGetPr(plhs[0]);

	// Remplir l'output
    for (unsigned int i=0; i<nQ*nQ; ++i)
			mass[i] = Mass(i);
			
	return;
}
