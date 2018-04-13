#ifndef S2MMUSCODINTERFACE_H
#define S2MMUSCODINTERFACE_H

    #include <cmath>
    #include <iostream>
    #include <eigen3/Eigen/Dense>
    #include <rbdl/rbdl.h>
//    #include "def_usrmod.hpp"


const unsigned int ndof = 19;			/** \brief The number of degrees of freedom for the model */
const unsigned int ncontrols = 13;		/** \brief The number of controls in the model */
const unsigned int nroot = 6;



namespace e = Eigen;
namespace s = std;
//namespace RBD = RigidBodyDynamics;
//namespace RBDM = RigidBodyDynamics::Math;

unsigned int  NMOS   = 1;  /* Number of phases (MOdel Stages) */
unsigned int  NP     = 0;  /* Number of parameters */
unsigned int  NRC    = 0;  /* Number of coupled constraints */
unsigned int  NRCE   = 0;  /* Number of coupled equality constraints */

unsigned int  NXD    = ndof * 2; /* Number of differential states */
unsigned int  NXA    = 0;        /* Number of algebraic states */
unsigned int  NU     = ncontrols;/* Number of controls */
unsigned int  NPR    = 0;        /* Number of local parameters */

unsigned int  NRD_S  = 0;        /* Number of constraints at the start point */
unsigned int  NRDE_S = 0;        /* Number of equality constraints at the start points */

unsigned int  NRD_E  = 0;        /* Number of constraints at the end point */
unsigned int  NRDE_E = 0;        /* Number of equality constraints at the end point */



/** \brief Right hand side of the differential equation */
static void ffcn_free (double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, long *info)
{
	e::VectorXd Q     (ndof);//shoulder->dof_count);
 	e::VectorXd QDot  (ndof);//shoulder->dof_count);
	e::VectorXd Tau   (ndof);//shoulder->dof_count);
	e::VectorXd QDDot (ndof);//shoulder->dof_count);


	// copy the values from xd
	for (unsigned int i = 0; i < ndof; i++) {
		Q   [i] = xd[i];
		QDot[i] = xd[i + ndof];
	}

	// set tau to zero and only set the values of the actuated joints
	// (remember: the first nroor dof are not actuated)
	Tau.setZero();
	for (unsigned int i = 0; i < ncontrols; i++){
		unsigned int j=i+nroot;
		Tau[j]=u[i];
	}
	assert (!isnan (   Q.norm()));
	assert (!isnan (QDot.norm()));
	assert (!isnan ( Tau.norm()));

	//RBD::ForwardDynamics (*shoulder, Q, QDot, Tau, QDDot);
	assert (!isnan (QDDot.norm()));

	// here we return
	//   ( qdot  )
	//   ( qddot )
	for (unsigned int i = 0; i < ndof; i++){//shoulder->dof_count; i++) {
		rhs[i]        = QDot (i);
		rhs[i + ndof] = QDDot(i);
	}
}




/** \brief Least-square function */
void lsqfcn(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, long *info)
{
    long i;
    long ncmsnode;


    if (*dpnd) {
    	*dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
    	return;
    }

    get_cnode(&ncmsnode);
    // ncmsnode = info->cnode;	// <-- this will be made active after merge of branch

		for (i = 0; i <  ndof; i++)
			res[i] = sd[i]- 0;
}



/** \brief Entry point for the muscod application */
extern "C" void def_model(void)
{
	RigidBodyDynamics::Model shoulder;

  /* Define problem dimensions */
  def_mdims(NMOS, NP, NRC, NRCE);

  def_mstage(
	0,
	NXD, NXA, NU,
	NULL, NULL,
	0, 0, 0, NULL, ffcn_free, NULL,
	NULL, NULL
	);

#ifdef LSQ
  def_lsq(0,  "*", 0, ndof, lsqfcn); //ndof = size of the vector in lsqfcn.
#endif

}

#endif // S2MMUSCODINTERFACE_H
