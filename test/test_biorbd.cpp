#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "s2mMusculoSkeletalModel.h"
#include "biorbdConfig.h"
#include "Utils/GenCoord.h"
#include "Utils/Tau.h"

static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
TEST(c3dFileIO, OpenModel){
#ifdef MODULE_ACTUATORS
    EXPECT_NO_THROW(s2mMusculoSkeletalModel model(modelPathForGeneralTesting));
#else // MODULE_ACTUATORS
    EXPECT_THROW(s2mMusculoSkeletalModel model(modelPathForGeneralTesting), std::runtime_error);
#endif // MODULE_ACTUATORS
}

static std::string modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
TEST(c3dConstraint, loopConstraint){
    s2mMusculoSkeletalModel model(modelPathForLoopConstraintTesting);
    biorbd::utils::GenCoord Q(model), QDot(model), QDDot_constrained(model), QDDot_expected(model);
    s2mTau Tau(model);
    Q.setZero();
    QDot.setZero();
    QDDot_constrained.setZero();
    QDDot_expected << 0.35935119225397999,  -10.110263945851218,  -6.0044250056047249e-14, -27.780105329642453, 31.783042765424835, 44.636461505611521,
            0.67809701484785911, -6.251969045745929e-16, 5.9580819677934563e-15, 15.822263679783546, 2.4880587040480365e-15, -9.930136612989094e-15;
    Tau.setZero();

    s2mContacts& cs(model.getConstraints_nonConst(model));
    RigidBodyDynamics::ForwardDynamicsConstraintsDirect(model, Q, QDot, Tau, cs, QDDot_constrained);
    for (unsigned int i = 0; i<model.nbQddot(); ++i)
        EXPECT_DOUBLE_EQ(QDDot_constrained[i], QDDot_expected[i]);
}
