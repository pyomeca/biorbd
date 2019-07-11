#define BIORBD_API_EXPORTS
#include "../include/IpoptTestMain.h"


// Copyright (C) 2005, 2009 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-10

// [MAIN]
#include "IpIpoptApplication.hpp"
#include <iostream>



mainTest::mainTest(
        s2mMusculoSkeletalModel &m
        ):
    m_model(m)
{}

mainTest::~mainTest(){}

int mainTest::main()
{
    // Create a new instance of your nlp
   //  (use a SmartPtr, not raw)
    s2mGenCoord Q(m_model), Qdot(m_model);
    Q.setZero();
    Q[0] = 0.3;
    Q[1] = 1.57;
    Qdot.setZero();
    Qdot[0] = 10;
    Qdot[1] = 15;
    std::vector<s2mMuscleStateActual> s;
    s.push_back(s2mMuscleStateActual(0, 0.2));
    s.push_back(s2mMuscleStateActual(0, 0.5));
    s.push_back(s2mMuscleStateActual(0, 0.7));
    s.push_back(s2mMuscleStateActual(0, 0.4));
    s.push_back(s2mMuscleStateActual(0, 0.3));
    s.push_back(s2mMuscleStateActual(0, 0.6));
    m_model.updateMuscles(m_model, Q, Qdot, true);
    s2mTau target = m_model.muscularJointTorque(m_model, s, false, &Q, &Qdot);

    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new HS071_NLP(m_model, Q, Qdot, target, true, 1);

    // Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    // Change some options
    // Note: The following choices are only examples, they might not be
    //       suitable for your optimization problem.
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetStringValue("derivative_test", "first-order");
    app->Options()->SetStringValue("check_derivatives_for_naninf", "yes");
    // app->Options()->SetIntegerValue("max_iter", 10000);
    // The following overwrites the default name (ipopt.opt) of the options file
    // app->Options()->SetStringValue("option_file_name", "hs071.opt");

    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Ipopt::Solve_Succeeded )
    {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        return static_cast<int>(status);
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    if( status == Ipopt::Solve_Succeeded )
    {
        std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
    }
    else
    {
        std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
    }

    // As the SmartPtrs go out of scope, the reference count
    // will be decremented and the objects will automatically
    // be deleted.

    return static_cast<int>(status);
}



