#define SKIP_FOR_REMOVING_DLIB
#ifndef SKIP_FOR_REMOVING_DLIB

#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleOptimisation.h"

s2mMuscleOptimisation::s2mMuscleOptimisation(s2mMusculoSkeletalModel& m, const s2mKalmanRecons::s2mKalmanParam &kalmanParam) :
    m_x0(.5){

    // Optimisation qui trouve les parametres modificateurs sur l'EMG pour fitter une dynamique inverse
    // S'assurer que tous les muscles envoyés sont de type HillTypeMaxime
    for (unsigned int i=0; i<m.nbMuscleGroups(); ++i) // groupe musculaire
        for (unsigned int j=0; j<m.muscleGroup(i).nbMuscles(); ++j)
            assert("All muscles should be of HillMax type" && !m.muscleGroup(i).muscle(j)->type().tolower().compare("hillmax"));

    // Préparation des variables
    m_kalman = new s2mKalmanReconsMarkers(m, kalmanParam);


}
s2mMuscleOptimisation::~s2mMuscleOptimisation(){
    delete m_kalman;
}


// This function is the "residual" for a least squares problem.   It takes an input/output
// pair and compares it to the output of our model and returns the amount of error.  The idea
// is to find the set of parameters which makes the residual small on all the data pairs.
double s2mMuscleOptimisation::residual (const OptimData& data,
                 const parameter_vector& x){
    // Calcul de la force musculaire
    unsigned int cmpMus(0);
    for (unsigned int i=0; i<data.m->nbMuscleGroups(); ++i) // groupe musculaire
        for (unsigned int j=0; j<data.m->muscleGroup(i).nbMuscles(); ++j){
            std::shared_ptr<s2mMuscleHillTypeMaxime> tp = std::static_pointer_cast<s2mMuscleHillTypeMaxime>(data.m->muscleGroup(i).muscle(j));
            tp->muscleGain(x(0));
            cmpMus++;
        }


    // Calcul du residu entre Tau_ID et Tau_mus
    double res(0);
    for (unsigned int i=0; i<data.Q().size(); ++i){
        s2mGenCoord Q(data.Q(i));
        s2mGenCoord QDot(data.QDot(i));
        s2mGenCoord muscleTorque(data.m->muscularJointTorque(*data.m, data.state(i), true, &Q, &QDot));

        s2mGenCoord Tau = data.Tau(i);
        res += (muscleTorque(data.dofFlex())-Tau(data.dofFlex())) * (muscleTorque(data.dofFlex())-Tau(data.dofFlex()));
    }
    std::cout << "x(0) = " << x(0) << std::endl;
    std::cout << "res = " << res << std::endl;
    return res;
}

// ----------------------------------------------------------------------------------------


double s2mMuscleOptimisation::optimizeJointTorque(s2mMusculoSkeletalModel& m, const s2mGenCoord& T, const std::vector<s2mMuscleStateActual>& s, unsigned int dofFlex){

    // Calcul de la cinématique inverse
    std::vector<s2mGenCoord> Q;
    std::vector<s2mGenCoord> QDot;
    std::vector<s2mGenCoord> QDDot;
    std::vector<s2mGenCoord> Tau;
    s2mGenCoord Q_tp(m.dof_count);
    s2mGenCoord QDot_tp(m.dof_count);
    s2mGenCoord QDDot_tp(m.dof_count);
    m_kalman->reconstructFrame(m,T,&Q_tp,&QDot_tp,&QDDot_tp);

    // Calcul de la dynamique inverse sur le modele
    s2mGenCoord Tau_tp(m.dof_count);
    RigidBodyDynamics::InverseDynamics(m,Q_tp, QDot_tp, QDDot_tp,Tau_tp);

    // Faire les pushBack
    Q.push_back(Q_tp);
    QDot.push_back(QDot_tp);
    QDDot.push_back(QDDot_tp);
    Tau.push_back(Tau_tp);
    std::vector<std::vector<s2mMuscleStateActual> > s_all;
    s_all.push_back(s);

    // Stocker toutes les variables envoyées
    std::vector<OptimData> data_samples;
    OptimData tp(m, Q, QDot, QDDot, Tau, s_all, dofFlex);
    data_samples.push_back(tp);


    // Now let's use the solve_least_squares_lm() routine to figure out what the
    // parameters are based on just the data_samples.
    parameter_vector x;
    x(0) = m_x0;

    // Use Levenberg-Marquardt, approximate derivatives
    dlib::solve_least_squares_lm(dlib::objective_delta_stop_strategy(1e-7,60),
                                     residual,
                                     dlib::derivative(residual),
                                     data_samples,
                                     x);

    m_x0 = x(0);
    return 1;
}

void s2mMuscleOptimisation::optimizeJointTorque(std::vector<OptimData>& data_samples, parameter_vector& x){
    // Use Levenberg-Marquardt, approximate derivatives
    dlib::solve_least_squares_lm(dlib::objective_delta_stop_strategy(1e-7,60),
                                     residual,
                                     dlib::derivative(residual),
                                     data_samples,
                                     x);
}





#endif

