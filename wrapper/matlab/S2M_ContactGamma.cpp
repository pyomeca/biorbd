
void S2M_ContactGamma( int nlhs, mxArray *plhs[],
				int nrhs, const mxArray*prhs[] ){
	// Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4, "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is the Qdot");
	
	// Recevoir le model
	s2mMusculoSkeletalModel * model = convertMat2Ptr<s2mMusculoSkeletalModel>(prhs[1]);
        unsigned int nQ = model->nbQ(); /* Get the number of DoF */ /**** ATTENTION, NQ A REMPLACÉ NDDL, SEGFAULT? ****/
        unsigned int nQdot = model->nbQdot(); /* Get the number of DoF */ /**** ATTENTION, NQ A REMPLACÉ NDDL, SEGFAULT? ****/

        // Recevoir Q
    s2mGenCoord Q = *getParameterQ(prhs, 2, nQ).begin();
    s2mGenCoord QDot = *getParameterQdot(prhs, 3, nQdot).begin();
    unsigned int nContacts = model->nContacts();

//    // Compute gamma
//    RigidBodyDynamics::ConstraintSet CS =  model->getConstraints(*model);
//    unsigned int prev_body_id = 0;
//    Eigen::Vector3d prev_body_point = Eigen::Vector3d::Zero();
//    Eigen::Vector3d gamma_i = Eigen::Vector3d::Zero();

//    CS.QDDot_0.setZero();
//    UpdateKinematicsCustom (*model, NULL, NULL, &CS.QDDot_0);

//    for (unsigned int i = 0; i < CS.size(); i++) {
//        // only compute point accelerations when necessary
//        if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
//            gamma_i = CalcPointAcceleration (*model, Q, QDot, CS.QDDot_0, CS.body[i], CS.point[i], false);
//            prev_body_id = CS.body[i];
//            prev_body_point = CS.point[i];
//        }

//        // we also substract ContactData[i].acceleration such that the contact
//        // point will have the desired acceleration
//        CS.gamma[i] = CS.acceleration[i] - CS.normal[i].dot(gamma_i);
//    }
    Eigen::MatrixXd G_tp(Eigen::MatrixXd::Zero(nContacts,model->nbQ()));
    RigidBodyDynamics::CalcConstraintsJacobian(*model, Q, model->getConstraints(*model), G_tp, true);

    RigidBodyDynamics::Math::VectorNd Gamma = model->getConstraints(*model).gamma;

        /* Create a matrix for the return argument */
    plhs[0] = mxCreateDoubleMatrix( nContacts, 1, mxREAL);
    double *gamma = mxGetPr(plhs[0]);
    for (unsigned int j=0; j<nContacts; ++j){
        gamma[j] = Gamma[j];
    }
    return;
}


