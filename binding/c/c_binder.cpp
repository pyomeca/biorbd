#include "c_binder.h"

S2MLIBRARY_API s2mMusculoSkeletalModel* c_s2mMusculoSkeletalModel(const char* pathToModel){
	return new s2mMusculoSkeletalModel(s2mRead::readModelFile(s2mString(pathToModel)));
}
S2MLIBRARY_API void c_deleteS2mMusculoSkeletalModel(s2mMusculoSkeletalModel* model){
	delete model;
}
S2MLIBRARY_API void c_writeS2mMusculoSkeletalModel(s2mMusculoSkeletalModel* model, const char * path){
	s2mWriter::writeModel(*model, s2mPath(path));
}





// IMUs functions
S2MLIBRARY_API int c_nIMUs(s2mMusculoSkeletalModel* model){
    return model->nIMUs();
}
void c_addIMU(s2mMusculoSkeletalModel *model, const double *imuRT, const char *name, const char *parentName, bool technical, bool anatomical)
{
    s2mAttitude pos(dispatchRTinput(imuRT));
    model->addIMU(pos, s2mString(name), s2mString(parentName), technical, anatomical, model->GetBodyId(parentName));
}
S2MLIBRARY_API void c_meanIMU(const double *imuRT, unsigned int nFrame, double* imuRT_mean)
{
    std::vector<s2mAttitude> m;

    // Dispatch des données d'entrée
    for (unsigned int i=0; i<nFrame; ++i) // Pour tous les instants
        m.push_back(dispatchRTinput(&imuRT[i*16]));

    // Calcul de la moyenne
    s2mAttitude mMean = s2mAttitude::mean(m);

    // Dispatch des données de sortie
    dispatchRToutput(mMean, imuRT_mean);
}
S2MLIBRARY_API s2mKalmanReconsIMU* c_s2mKalmanReconsIMU(s2mMusculoSkeletalModel* model, double* QinitialGuess, double freq, double noiseF, double errorF){
	// Créer un pointeur sur un filtre de kalman
	s2mKalmanReconsIMU* kalman = new s2mKalmanReconsIMU(*model, s2mKalmanRecons::s2mKalmanParam(freq, noiseF, errorF));
	
	// Mettre le initial guess
	s2mGenCoord e_QinitialGuess(*model);
	if (QinitialGuess != NULL){
		for (size_t i = 0; i<model->nbQ(); ++i)
			e_QinitialGuess[i] = QinitialGuess[i];
		kalman->setInitState(&e_QinitialGuess);
	}
		
	return kalman;
}
S2MLIBRARY_API void c_deleteS2mKalmanReconsIMU(s2mKalmanReconsIMU* model){
	delete model;
}
S2MLIBRARY_API void c_s2mKalmanReconsIMUstep(s2mMusculoSkeletalModel* model, s2mKalmanReconsIMU* kalman, double* imu, double* Q, double* QDot, double* QDDot){
	// Copier les valeurs des matrices de rotation des IMUs dans un stl Vector
    Eigen::VectorXd T(3*3*model->nIMUs()); // Matrice 3*3 * nIMU
    for (unsigned int i=0; i<model->nIMUs(); ++i)
        for (unsigned int j=0; j<9; ++j){ // matrice 3*3
			T[9*i+j] = imu[9*i+j];
		}
	
	// Se faire des entrés sur Q, QDot et QDDot
	s2mGenCoord e_Q(*model), e_QDot(*model), e_QDDot(*model);

	// Faire le filtre
	kalman->reconstructFrame(*model, T, &e_Q, &e_QDot, &e_QDDot);
	
    // Transcrire les réponses vers les arguments de sortie
    dispatchQoutput(e_Q, Q);
    dispatchQoutput(e_QDot, QDot);
    dispatchQoutput(e_QDDot, QDDot);
}




// Joints functions
S2MLIBRARY_API void c_globalJCS(s2mMusculoSkeletalModel* m, const double* Q, double* jcs){
	// Dispatch des données d'entrée
    s2mGenCoord eQ = dispatchQinput(m, Q);

	// Récupérer JCS
	std::vector<s2mAttitude> pre_jcs = m->globalJCS(eQ);

	// Dispatch de l'output
    dispatchRToutput(pre_jcs, jcs);
}
S2MLIBRARY_API void c_projectJCSinParentBaseCoordinate(const double* parent, const double* jcs, double * out){
    // Recueillir les données d'entrée
    s2mAttitude aParent(dispatchRTinput(parent));
    s2mAttitude aJcs(dispatchRTinput(jcs));

    // Projeter et préparer les données de sortie
    dispatchRToutput(aParent.transpose() * aJcs, out);
}
S2MLIBRARY_API void c_matrixMultiplication(const double* M1, const double* M2, double* Mout){
	// Recueillir les données d'entrée
    s2mAttitude mM1(dispatchRTinput(M1));
    s2mAttitude mM2(dispatchRTinput(M2));
	
    // Projeter et préparer les données de sortie
    dispatchRToutput(mM1 * mM2, Mout);
}
S2MLIBRARY_API void c_localJCS(s2mMusculoSkeletalModel* m, int i, double* rt_out)
{
    s2mAttitude RT(m->bone(i).localJCS());
    dispatchRToutput(RT, rt_out);
}
S2MLIBRARY_API void c_boneRotationSequence(s2mMusculoSkeletalModel* m, const char* segName, char* seq) {
    s2mString sequence(m->bone(segName).seqR());
    strcpy(seq, sequence.c_str()); // On assume que la mémoire de seq est déjà allouée
}




// dof functions
S2MLIBRARY_API int c_nQ(s2mMusculoSkeletalModel* model){
	return model->nbQ();
}
S2MLIBRARY_API int c_nQDot(s2mMusculoSkeletalModel* model){
	return model->nbQdot();
}
S2MLIBRARY_API int c_nQDDot(s2mMusculoSkeletalModel* model){
	return model->nbQddot();
}
S2MLIBRARY_API int c_nTau(s2mMusculoSkeletalModel* model){
	return model->nbTau();
}







// Markers functions
S2MLIBRARY_API int c_nTags(s2mMusculoSkeletalModel* model){
	return model->nTags(); 
}
S2MLIBRARY_API void c_Tags(s2mMusculoSkeletalModel* model, const double* Q, double* markPos, bool removeAxis, bool updateKin){
	// Prepare parameters
    s2mGenCoord eQ = dispatchQinput(model, Q);
	
	// Call the main function
	std::vector<s2mNodeBone> pos(model->Tags(*model, eQ, removeAxis, updateKin));

	// Prepare output
    dispatchTagsOutput(pos, markPos);
}
S2MLIBRARY_API void c_TagsInLocal(s2mMusculoSkeletalModel* model, double* markPos){
	// Prepare output
    dispatchTagsOutput(model->Tags(), markPos);
}
S2MLIBRARY_API void c_addTags(s2mMusculoSkeletalModel *model, const double *markPos, const char* name, const char* parentName, bool technical, bool anatomical, const char* axesToRemove)
{
    int parent_int = model->GetBodyId(parentName);
    Eigen::Vector3d pos(dispatchTagsInput(markPos)); // Position du marker dans le repère local
    model->addMarker(pos, s2mString(name), s2mString(parentName), technical, anatomical, s2mString(axesToRemove), parent_int);
}


S2MLIBRARY_API void c_transformMatrixToCardan(const double *M, const char *sequence, double* cardanOut)
{
    s2mAttitude mM(dispatchRTinput(M));
    s2mString seq(sequence);

    Eigen::VectorXd cardan(s2mAttitude::transformMatrixToCardan(mM, seq));

    // On assume que la mémoire pour cardanOut a déjà été octroyée
    dispatchDoubleOutput(cardan, cardanOut);
}


// Fonctions de dispatch des données d'entré ou de sortie
Eigen::Vector3d dispatchTagsInput(const double * pos){
    return Eigen::Vector3d(pos[0], pos[1], pos[2]);
}
void dispatchTagsOutput(const std::vector<s2mNodeBone> &allTags, double* tags){
    // Attention tags doit déjà avoir sa mémoire d'allouée
    for (size_t i = 0; i<allTags.size(); ++i){
        for (size_t j = 0; j<3; ++j){
            tags[i*3+j] = (*(allTags.begin()+i))[j];
        }
    }
}
s2mGenCoord dispatchQinput(s2mMusculoSkeletalModel* model, const double*Q){
	// Prepare parameters
	Eigen::VectorXd eQ = Eigen::VectorXd(model->nbQ());
	for (size_t i = 0; i < model->nbQ(); ++i){
		eQ[i] = Q[i];
	}
    return eQ;
}
void dispatchQoutput(const s2mGenCoord &eQ, double*Q){
    // ATTENTION Q DOIT DÉJÀ AVOIR SA MÉMOIRE ALLOUÉE
    for (unsigned int i=0; i<eQ.size(); ++i){
        Q[i] = eQ[i];
    }
}
void dispatchDoubleOutput(const Eigen::VectorXd& e, double* d){
    // ATTENTION Q DOIT DÉJÀ AVOIR SA MÉMOIRE ALLOUÉE
    for (unsigned int i=0; i<e.size(); ++i){
        d[i] = e[i];
    }
}
s2mAttitude dispatchRTinput(const double* rt){
    s2mAttitude pos;
    pos <<  rt[0], rt[4], rt[8], rt[12],
            rt[1], rt[5], rt[9], rt[13],
            rt[2], rt[6], rt[10], rt[14],
            rt[3], rt[7], rt[11], rt[15];
    return pos;
}
void dispatchRToutput(const s2mAttitude& rt_in, double* rt_out){
    // Attention la mémoire doit déjà être allouée pour rt_out
    for (unsigned int i=0; i<16; ++i){
        rt_out[i] = rt_in(i%4, i/4);
    }
}
void dispatchRToutput(const std::vector<s2mAttitude>& rt_in, double* rt_out){
    // Attention la mémoire doit déjà être allouée pour rt_out
    for (unsigned int i=0; i<rt_in.size(); ++i){
        for (unsigned int j=0; j<16; ++j){
            rt_out[i*16+j] = rt_in[i](j%4, j/4);
        }
    }
}








// Spécifique à des projets (IMU sous Unity)
S2MLIBRARY_UNITY_API void c_alignSpecificAxisWithParentVertical(const double* parentRT, const double * childRT, int idxAxe, double * rotation){
	// Matrices à aliger (axe 2 de r1 avec axe idxAxe de r2)
	s2mAttitude r1(dispatchRTinput(parentRT));
	s2mAttitude r2(dispatchRTinput(childRT));

    s2mAttitude rotationMat = s2mIMU_Unity_Optim::alignSpecificAxisWithParentVertical(r1, r2, idxAxe);

    dispatchRToutput(rotationMat, rotation);
}

