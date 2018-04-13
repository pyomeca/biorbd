void S2M_parent( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] ){
	
	// Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3, "3 arguments are required where the 2nd is the handler on the model and the 3rd is the name of the segment");
	s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
					  
	/* Create a matrix for the return argument */ 
	plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL); 

    /* Quel segment */
    int idx = getInteger(prhs, 2, "index");

    if (idx<1){
        std::ostringstream msg;
        msg << "Segment index must be 1 or higher.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }
    else if  (idx>model->nbBone()){
        std::ostringstream msg;
        msg << "Segment index must not be higher than number of segment.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    // Sortie du nom du segment parent
    plhs[0] = mxCreateString ( model->GetBodyName(model->bone(idx-1).parent_rbdl_id()).c_str()); // Recueillir le nom

	return;
}
