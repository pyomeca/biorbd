#ifndef S2MSTATICOPTIMIZATIONIPOPT_H
#define S2MSTATICOPTIMIZATIONIPOPT_H

#include <IpIpoptApplication.hpp>
#include <iostream>
#include "IpTNLP.hpp"
#include "biorbdConfig.h"
#include "s2mGenCoord.h"
#include "s2mMusculoSkeletalModel.h"
#include "s2mMuscles.h"
#include "s2mVector.h"
#include "s2mMuscleStateActual.h"

#include <Eigen/Dense>


    
class BIORBD_API s2mStaticOptimizationIpopt : public Ipopt::TNLP
{
    public:
        s2mStaticOptimizationIpopt(s2mMusculoSkeletalModel &model,
                const s2mGenCoord           &Q,
                const s2mGenCoord           &Qdot,
                const s2mTau                &tauTarget,
                const s2mVector             &activationInit,
                bool                        useResidual = true,
                unsigned int                pNormFactor = 2,
                int                         verbose = 0,
                const double                eps = 1e-10
                );

        virtual ~s2mStaticOptimizationIpopt();

        /** Method to return some info about the NLP */
        virtual bool get_nlp_info(
           Ipopt::Index&          n,
           Ipopt::Index&          m,
           Ipopt::Index&          nnz_jac_g,
           Ipopt::Index&          nnz_h_lag,
           IndexStyleEnum& index_style
           );

        /** Method to return the bounds for my problem */
        virtual bool get_bounds_info(
           Ipopt::Index   n,
           Ipopt::Number* x_l,
           Ipopt::Number* x_u,
           Ipopt::Index   m,
           Ipopt::Number* g_l,
           Ipopt::Number* g_u
           );

        /** Method to return the starting point for the algorithm */
        virtual bool get_starting_point(
           Ipopt::Index   n,
           bool    init_x,
           Ipopt::Number* x,
           bool    init_z,
           Ipopt::Number* z_L,
           Ipopt::Number* z_U,
           Ipopt::Index   m,
           bool    init_lambda,
           Ipopt::Number* lambda
           );

        /** Method to return the objective value */
        virtual bool eval_f(
           Ipopt::Index         n,
           const Ipopt::Number* x,
           bool                 new_x,
           Ipopt::Number&       obj_value
           );

        /** Method to return the gradient of the objective */
        virtual bool eval_grad_f(
           Ipopt::Index         n,
           const Ipopt::Number* x,
           bool          new_x,
           Ipopt::Number*       grad_f
           );

        /** Method to return the constraint residuals */
        virtual bool eval_g(
           Ipopt::Index         n,
           const Ipopt::Number* x,
           bool          new_x,
           Ipopt::Index         m,
           Ipopt::Number*       g
           );

        /** Method to return:
         *   1) The structure of the jacobian (if "values" is NULL)
         *   2) The values of the jacobian (if "values" is not NULL)
         */
        virtual bool eval_jac_g(
           Ipopt::Index         n,
           const Ipopt::Number* x,
           bool                 new_x,
           Ipopt::Index         m,
           Ipopt::Index         nele_jac,
           Ipopt::Index*        iRow,
           Ipopt::Index*        jCol,
           Ipopt::Number*       values
           );

        virtual void finalize_solution(
           Ipopt::SolverReturn               status,
           Ipopt::Index                      n,
           const Ipopt::Number*              x,
           const Ipopt::Number*              z_L,
           const Ipopt::Number*              z_U,
           Ipopt::Index                      m,
           const Ipopt::Number*              g,
           const Ipopt::Number*              lambda,
           Ipopt::Number                     obj_value,
           const Ipopt::IpoptData*           ip_data,
           Ipopt::IpoptCalculatedQuantities* ip_cq
           );

        s2mVector finalSolution(){return m_finalSolution;}
        s2mVector finalResidual(){return m_finalResidual;}


    protected:
        s2mMusculoSkeletalModel &m_model;
        unsigned int m_nQ;
        unsigned int m_nQdot;
        unsigned int m_nMus;
        unsigned int m_nDof;
        unsigned int m_nTau;
        unsigned int m_nTauResidual;
        double m_eps;
        s2mVector m_activations;
        s2mGenCoord m_Q;
        s2mGenCoord m_Qdot;        
        s2mTau m_tauTarget;
        s2mVector m_tauResidual;
        double m_tauPonderation;
        std::vector<s2mMuscleStateActual> m_states;
        unsigned int m_pNormFactor;
        int m_verbose;
        s2mVector m_finalSolution;
        s2mVector m_finalResidual;

        void dispatch(
                const Ipopt::Number* x
                );

    private:

};
#endif // S2MSTATICOPTIMIZATIONIPOPT_H
