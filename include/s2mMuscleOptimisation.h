#ifndef S2MMUSCLEOPTIMISATION_H
#define S2MMUSCLEOPTIMISATION_H


    #include <dlib/optimization.h>
    #include "s2mMusculoSkeletalModel.h"
    #include "s2mMuscleState.h"
    #include "s2mKalmanReconsMarkers.h"



class s2mGenCoord;
class s2mMusculoSkeletalModel;
class s2mMuscleOptimisation
{

    public:
        typedef dlib::matrix<double,1,1> parameter_vector;
        class OptimData{
        public:
            OptimData(s2mMusculoSkeletalModel &m,
                      const std::vector<s2mGenCoord>& Q,
                      const std::vector<s2mGenCoord>& Qdot,
                      const std::vector<s2mGenCoord>& Qddot,
                      const std::vector<s2mGenCoord>& Tau,
                      const std::vector<std::vector<s2mMuscleStateActual> > state,
                      const unsigned int dofFlex) :
                m(&m),m_Q(Q),m_QDot(Qdot),m_QDDot(Qddot),m_Tau(Tau),m_s(state),m_dofFlex(dofFlex){

            }

            ~OptimData(){}
            s2mMusculoSkeletalModel * m;
            std::vector<s2mGenCoord> Q() const {return m_Q;}
            s2mGenCoord Q(unsigned int i) const {return m_Q[i];}
            std::vector<s2mGenCoord> QDot() const {return m_QDot;}
            s2mGenCoord QDot(unsigned int i) const {return m_QDot[i];}
            std::vector<s2mGenCoord> QDDot() const {return m_QDDot;}
            s2mGenCoord QDDot(unsigned int i) const {return m_QDDot[i];}
            std::vector<s2mGenCoord> Tau() const {return m_Tau;}
            s2mGenCoord Tau(unsigned int i) const {return m_Tau[i];}
            std::vector<std::vector<s2mMuscleStateActual> > state() const {return m_s;}
            std::vector<s2mMuscleStateActual> state(unsigned int i) const {return m_s[i];}
            unsigned int dofFlex() const {return m_dofFlex;}

        private:

            std::vector<s2mGenCoord> m_Q;
            std::vector<s2mGenCoord> m_QDot;
            std::vector<s2mGenCoord> m_QDDot;
            std::vector<s2mGenCoord> m_Tau;
            std::vector<std::vector<s2mMuscleStateActual> > m_s;
            unsigned int m_dofFlex;
        };

        s2mMuscleOptimisation(s2mMusculoSkeletalModel& m, const s2mKalmanRecons::s2mKalmanParam &kalmanParam);
        virtual ~s2mMuscleOptimisation();

        /* Lunch an optimization from a model, with Q, Qdot, Qddot and muscleState */
        double optimizeJointTorque(s2mMusculoSkeletalModel&, const s2mGenCoord&, const std::vector<s2mMuscleStateActual>&, unsigned int);
        static void optimizeJointTorque(std::vector<OptimData>& d, parameter_vector& x);



    protected:
        static double residual (const OptimData& data, const parameter_vector& x); // Optimization

        s2mKalmanReconsMarkers *m_kalman;
        double m_x0; // Solutions initiales

};

#endif // S2MMUSCLEOPTIMISATION_H
