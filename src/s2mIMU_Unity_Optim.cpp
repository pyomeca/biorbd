#define BIORBD_API_EXPORTS
#include "../include/s2mIMU_Unity_Optim.h"

s2mIMU_Unity_Optim::OptimData::OptimData(const s2mAttitude &R1, const s2mAttitude &R2, int axe)
       : m_R1(R1), m_R2(R2), m_axe(axe)
{

}


double s2mIMU_Unity_Optim::residual(const OptimData &data, const parameter_vector &x){
	Eigen::VectorXd rotation(1);
	rotation(0) = x;
	s2mAttitude toRot(rotation, Eigen::Vector3d::Zero(), "z");
	s2mAttitude R2prime = toRot * data.m_R2;
	
	//  Extraire les colonnes à trouver le produit scalaire
    Eigen::Vector3d axisR2 = R2prime.block(0,data.m_axe,3,1);
	Eigen::Vector3d zAxisR1 = data.m_R1.block(0,2,3,1);

	// Faire le produit scalaire
	double dotProd(axisR2.dot(zAxisR1));

	// Retourner l'absolu
    return fabs(dotProd)-1;
}

s2mAttitude s2mIMU_Unity_Optim::alignSpecificAxisWithParentVertical(const s2mAttitude &r1, const s2mAttitude &r2, int idxAxe){
	// Matrices à aliger (axe 2 (z) de r1 avec axe idxAxe de r2)

	// Préparer l'optimisation (paramètre)
    std::vector<OptimData> data;
    data.push_back(OptimData(r1, r2, idxAxe));
	
	// Variable à optimiser (initial guess)
    parameter_vector x;
    x(0) = 0;

    // Use Levenberg-Marquardt, approximate derivatives
    dlib::solve_least_squares_lm(dlib::objective_delta_stop_strategy(1e-10).be_verbose(),
                                     residual,
                                     dlib::derivative(residual),
                                     data,
                                     x);

    // Dispatch de la matrice optimale de passage
    Eigen::VectorXd rotation(1);
    rotation(0) = x(0);

    return s2mAttitude (rotation, Eigen::Vector3d::Zero(), "z");
}