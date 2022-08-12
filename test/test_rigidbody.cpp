#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>
#include <string.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Utils/Range.h"
#include "Utils/SpatialVector.h"
#include "Utils/Matrix3d.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/SoftContactSphere.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/IMU.h"
#ifdef MODULE_KALMAN
    #include "RigidBody/KalmanReconsMarkers.h"
    #include "RigidBody/KalmanReconsIMU.h"
#endif

#include "testWrapperClass.h"

using namespace BIORBD_NAMESPACE;

static double requiredPrecision(1e-10);
#ifdef MODULE_ACTUATORS
    static std::string
    modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
    static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
static std::string modelPathMeshEqualsMarker("models/meshsEqualMarkers.bioMod");
static std::string
modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
static std::string modelNoRoot("models/pyomecaman_freeFall.bioMod");
static std::string modelNoRootDoF("models/pyomecaman_stuck.bioMod");
static std::string modelSimple("models/cube.bioMod");

static std::string modelWithSoftContact("models/cubeWithSoftContacts.bioMod");
static std::string modelWithRigidContactsExternalForces("models/cubeWithRigidContactsExternalForces.bioMod");
static std::string modelWithSoftContactRigidContactsExternalForces("models/cubeWithSoftContactsRigidContactsExternalForces.bioMod");

TEST(Gravity, change)
{
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);
    // Set to random values
    std::vector<double> val(model.nbQ());
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);
    {
        CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);

        std::vector<double> QDDot_expected(13);
        QDDot_expected[1] = -9.81;
        for (unsigned int i = 0; i<model.nbQddot(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                        requiredPrecision);
        }
    }

    model.setGravity(utils::Vector3d(0, -2.2, 0));
    {
        CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);
        std::vector<double> QDDot_expected(13);
        QDDot_expected[0] = -2.2;
        for (unsigned int i = 0; i<model.nbQddot(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                        requiredPrecision);
        }
    }
}

TEST(Contacts, unitTest)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Contacts contacts(model);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);
        EXPECT_STREQ(contacts.contactName(1).c_str(), "PiedG_1_Z");
        EXPECT_STREQ(contacts.contactNames()[1].c_str(), "PiedG_1_Z");
        EXPECT_EQ(contacts.hasContacts(), true);
        SCALAR_TO_DOUBLE(fy, contacts.getForce()[1]);
        EXPECT_NEAR(fy, 0., requiredPrecision);
    }
    {
        rigidbody::Contacts contacts;
        EXPECT_EQ(contacts.hasContacts(), false);
    }
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Contacts contacts(model);

        EXPECT_THROW(contacts.contactName(7), std::runtime_error);
    }
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Contacts contacts(model);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);

        contacts.AddConstraint(
            7,
            utils::Vector3d(0, 0, 0),
            utils::Vector3d(1, 1, 1),
            "constraintName");

        EXPECT_NEAR(contacts.nbContacts(), 7., requiredPrecision);
    }
    {
        Model model(modelPathForGeneralTesting);

        EXPECT_EQ(model.rigidContactAxisIdx(0)[0], 1);
        EXPECT_EQ(model.rigidContactAxisIdx(0)[1], 2);
    }
    {
        Model model(modelPathForGeneralTesting);

        EXPECT_EQ(model.contactSegmentBiorbdId(0), 7);
    }
    {
        Model model(modelPathForGeneralTesting);

        EXPECT_EQ(model.segmentRigidContactIdx(7)[0], 0);
        EXPECT_EQ(model.segmentRigidContactIdx(7)[1], 1);
    }
}

TEST(Contacts, computeForceAtOrigin)
{
    {
        Model model(modelPathForGeneralTesting);

        utils::Vector force;
        force = utils::Vector(2);
        force[0] = 1;
        force[1] = 2;

        const utils::SpatialVector forceAtOrigin = model.computeForceAtOrigin(utils::Vector3d(1, 1, 1),std::vector<int>({1, 2}),force);

        std::vector<double> forceExpected = {1, -2, 1,
                                             0, 1, 2};
        for (unsigned int i = 0; i < 6; ++i)
        {
            SCALAR_TO_DOUBLE(f, forceAtOrigin(i));
            EXPECT_NEAR(f, forceExpected[i], requiredPrecision);
        }
    }
}

TEST(Contacts, rigidContactToSpatialVector)
{
    {
        Model model(modelPathForGeneralTesting);

        rigidbody::GeneralizedCoordinates Q(model);
        FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2, 0.3,
                                           -2.01, -3.01, -3.01, 0.1, 0.2, 0.3, 0.4}));

        std::vector<utils::Vector> realAllForces;
        std::vector<utils::Vector> *allForces;
        allForces = &realAllForces;

        std::vector<RigidBodyDynamics::Math::SpatialVector>*
        SpatialVectorAtOriginNULL = model.rigidContactToSpatialVector(Q, allForces, true);

        EXPECT_FALSE(SpatialVectorAtOriginNULL);
    }
    {
        Model model(modelPathForGeneralTesting);

        rigidbody::GeneralizedCoordinates Q(model);
        FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2, 0.3,
                                           -2.01, -3.01, -3.01, 0.1, 0.2, 0.3, 0.4}));

        std::vector<utils::Vector> realAllForces;
        std::vector<utils::Vector> *allForces;
        allForces = &realAllForces;

        utils::Vector force1;
        force1 = utils::Vector(2);
        force1[0] = 1;
        force1[1] = 2;

        utils::Vector force2;
        force2 = utils::Vector(1);
        force2[0] = 3;

        utils::Vector force3;
        force3 = utils::Vector(2);
        force3[0] = 4;
        force3[1] = 5;

        utils::Vector force4;
        force4 = utils::Vector(1);
        force4[0] = 6;

        allForces->push_back(force1);
        allForces->push_back(force2);
        allForces->push_back(force3);
        allForces->push_back(force4);

        std::vector<RigidBodyDynamics::Math::SpatialVector>*
                SpatialVectorAtOrigin = model.rigidContactToSpatialVector(Q, allForces, true);

        RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
        RigidBodyDynamics::Math::SpatialVector sp_dof12(-7.592268755852077, -0.7864099999999999, 0.14995, 0, 1.0, 5.0);
        RigidBodyDynamics::Math::SpatialVector sp_dof15(-17.952947566495585, 1.72277, -0.5998, 0.0, 4.0, 11.0);

        std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

        sp_expected.push_back(sp_zero); // Dof 0
        sp_expected.push_back(sp_zero); // Dof 1
        sp_expected.push_back(sp_zero); // Dof 2
        sp_expected.push_back(sp_zero); // Dof 3
        sp_expected.push_back(sp_zero); // Dof 4
        sp_expected.push_back(sp_zero); // Dof 5
        sp_expected.push_back(sp_zero); // Dof 6
        sp_expected.push_back(sp_zero); // Dof 7
        sp_expected.push_back(sp_zero); // Dof 8
        sp_expected.push_back(sp_zero); // Dof 9
        sp_expected.push_back(sp_zero); // Dof 10
        sp_expected.push_back(sp_zero); // Dof 11
        sp_expected.push_back(sp_dof12); // Dof 12
        sp_expected.push_back(sp_zero); // Dof 13
        sp_expected.push_back(sp_zero); // Dof 14
        sp_expected.push_back(sp_dof15); // Dof 15

        for (unsigned int i = 0; i < 16; ++i){

            for (unsigned int j = 0; j < 6; ++j)
                    {
                        SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                        SCALAR_TO_DOUBLE(f, (*SpatialVectorAtOrigin)[i](j));
                        EXPECT_NEAR(f, f_expected, requiredPrecision);
                    }
        }
    }
}

TEST(Contacts, DeepCopy)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Contacts contacts(model);

        rigidbody::Contacts shallowCopy(contacts);
        rigidbody::Contacts deepCopyNow(contacts.DeepCopy());
        rigidbody::Contacts deepCopyLater;
        deepCopyLater.DeepCopy(contacts);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(shallowCopy.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyNow.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyLater.nbContacts(), 6., requiredPrecision);

        contacts.AddConstraint(
            7,
            utils::Vector3d(0, 0, 0),
            utils::Vector3d(1, 1, 1),
            "constraintName");

        EXPECT_NEAR(contacts.nbContacts(), 7., requiredPrecision);
        EXPECT_NEAR(shallowCopy.nbContacts(), 7., requiredPrecision);
        EXPECT_NEAR(deepCopyNow.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyLater.nbContacts(), 6., requiredPrecision);
    }
}

TEST(RigidContacts, create){
    Model model;
    model.AddSegment("Seg1", "root", "xyz", "xyz", {}, {}, {}, rigidbody::SegmentCharacteristics(), utils::RotoTrans());
    unsigned int id = model.GetBodyId("Seg1");
    model.AddConstraint(id, utils::Vector3d(0.1, 0.2, 0.3), utils::Vector3d(1, 0, 0), "ContactX");
    model.AddConstraint(id, utils::Vector3d(0.4, 0.5, 0.6), utils::Vector3d(0, 1, 0), "ContactY");
    model.AddConstraint(id, utils::Vector3d(0.7, 0.8, 0.9), utils::Vector3d(0, 0, 1), "ContactZ");
    model.AddConstraint(id, utils::Vector3d(0.1, 0.2, 0.3), "x", "ContactX2");

    EXPECT_EQ(model.nbRigidContacts(), 4);

    {
        unsigned int n(0);
        EXPECT_STREQ(model.rigidContact(n).utils::Node::name().c_str(), "ContactX");
        SCALAR_TO_DOUBLE(contactX, model.rigidContact(n)[0]);
        SCALAR_TO_DOUBLE(contactY, model.rigidContact(n)[1]);
        SCALAR_TO_DOUBLE(contactZ, model.rigidContact(n)[2]);
        EXPECT_EQ(contactX, 0.1);
        EXPECT_EQ(contactY, 0.2);
        EXPECT_EQ(contactZ, 0.3);
        EXPECT_EQ(model.rigidContact(n).axes()[0], 1);
        EXPECT_EQ(model.rigidContact(n).axes()[1], 0);
        EXPECT_EQ(model.rigidContact(n).axes()[2], 0);
    }

    {
        unsigned int n(1);
        EXPECT_STREQ(model.rigidContact(n).utils::Node::name().c_str(), "ContactY");
        SCALAR_TO_DOUBLE(contactX, model.rigidContact(n)[0]);
        SCALAR_TO_DOUBLE(contactY, model.rigidContact(n)[1]);
        SCALAR_TO_DOUBLE(contactZ, model.rigidContact(n)[2]);
        EXPECT_EQ(contactX, 0.4);
        EXPECT_EQ(contactY, 0.5);
        EXPECT_EQ(contactZ, 0.6);
        EXPECT_EQ(model.rigidContact(n).axes()[0], 0);
        EXPECT_EQ(model.rigidContact(n).axes()[1], 1);
        EXPECT_EQ(model.rigidContact(n).axes()[2], 0);
    }

    {
        unsigned int n(2);
        EXPECT_STREQ(model.rigidContact(n).utils::Node::name().c_str(), "ContactZ");
        SCALAR_TO_DOUBLE(contactX, model.rigidContact(n)[0]);
        SCALAR_TO_DOUBLE(contactY, model.rigidContact(n)[1]);
        SCALAR_TO_DOUBLE(contactZ, model.rigidContact(n)[2]);
        EXPECT_EQ(contactX, 0.7);
        EXPECT_EQ(contactY, 0.8);
        EXPECT_EQ(contactZ, 0.9);
        EXPECT_EQ(model.rigidContact(n).axes()[0], 0);
        EXPECT_EQ(model.rigidContact(n).axes()[1], 0);
        EXPECT_EQ(model.rigidContact(n).axes()[2], 1);
    }

    {
        unsigned int n(3);
        EXPECT_STREQ(model.rigidContact(n).utils::Node::name().c_str(), "ContactX2");
        SCALAR_TO_DOUBLE(contactX, model.rigidContact(n)[0]);
        SCALAR_TO_DOUBLE(contactY, model.rigidContact(n)[1]);
        SCALAR_TO_DOUBLE(contactZ, model.rigidContact(n)[2]);
        EXPECT_EQ(contactX, 0.1);
        EXPECT_EQ(contactY, 0.2);
        EXPECT_EQ(contactZ, 0.3);
        EXPECT_EQ(model.rigidContact(n).axes()[0], 1);
        EXPECT_EQ(model.rigidContact(n).axes()[1], 0);
        EXPECT_EQ(model.rigidContact(n).axes()[2], 0);
    }


    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity QDot(model);
    rigidbody::GeneralizedVelocity QDDot(model);
    FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2, 0.3}));
    FILL_VECTOR(QDot, std::vector<double>({0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
    FILL_VECTOR(QDDot, std::vector<double>({0.5, 0.5, 0.5, 0.5, 0.5, 0.5}));

    {
        std::vector<double> positionExpected = {-1.9146957599281647, -2.8191133387246508, -2.70262501017246};
        auto position = model.rigidContact(Q, 0, true);
        auto positionAll = model.rigidContacts(Q, true);
        for (unsigned int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(p, position[i]);
            SCALAR_TO_DOUBLE(p2, positionAll[0][i]);
            EXPECT_NEAR(p, positionExpected[i], requiredPrecision);
            EXPECT_NEAR(p2, positionExpected[i], requiredPrecision);
        }
    }

    {
        std::vector<double> velocityExpected = {0.10705609071138775, 0.0734011441522157, 0.11433067610957084};
        auto velocity = model.rigidContactVelocity(Q, QDot, 0);
        auto velocityAll = model.rigidContactsVelocity(Q, QDot);
        for (unsigned int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(v, velocity[i]);
            SCALAR_TO_DOUBLE(v2, velocityAll[0][i]);
            EXPECT_NEAR(v, velocityExpected[i], requiredPrecision);
            EXPECT_NEAR(v2, velocityExpected[i], requiredPrecision);
        }
    }

    {
        std::vector<double> accelerationExpected = {0.53484698259518959, 0.36370063728727758, 0.57070845052506336};
        auto acceleration = model.rigidContactAcceleration(Q, QDot, QDDot, 0);
        auto accelerationAll = model.rigidContactsAcceleration(Q, QDot, QDDot);
        for (unsigned int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(a, acceleration[i]);
            SCALAR_TO_DOUBLE(a2, accelerationAll[0][i]);
            EXPECT_NEAR(a, accelerationExpected[i], requiredPrecision);
            EXPECT_NEAR(a2, accelerationExpected[i], requiredPrecision);
        }
    }
}

TEST(SoftContacts, creation){
    {
        Model model(modelWithSoftContact);
        rigidbody::SoftContacts contacts(model);

        EXPECT_EQ(contacts.nbSoftContacts(), 2);
        EXPECT_STREQ(contacts.softContactName(1).c_str(), "Contact2");
        EXPECT_STREQ(contacts.softContactNames()[1].c_str(), "Contact2");

        const rigidbody::SoftContactNode& contact(contacts.softContact(0));
        const rigidbody::SoftContactSphere &sphere(dynamic_cast<const rigidbody::SoftContactSphere&>(contact));

        {
            SCALAR_TO_DOUBLE(radius, sphere.radius());
            EXPECT_NEAR(radius, 5, requiredPrecision);
        }
        {
            SCALAR_TO_DOUBLE(stiffness, sphere.stiffness());
            EXPECT_NEAR(stiffness, 6, requiredPrecision);
        }
        {
            SCALAR_TO_DOUBLE(damping, sphere.damping());
            EXPECT_NEAR(damping, 7, requiredPrecision);
        }

        {
            SCALAR_TO_DOUBLE(muStatic, sphere.muStatic());
            EXPECT_NEAR(muStatic, 0.8, requiredPrecision);
        }
        {
            SCALAR_TO_DOUBLE(muDynamic, sphere.muDynamic());
            EXPECT_NEAR(muDynamic, 0.7, requiredPrecision);
        }
        {
            SCALAR_TO_DOUBLE(muViscous, sphere.muViscous());
            EXPECT_NEAR(muViscous, 0.5, requiredPrecision);
        }
        {
            SCALAR_TO_DOUBLE(transitionVelocity, sphere.transitionVelocity());
            EXPECT_NEAR(transitionVelocity, 0.01, requiredPrecision);
        }
    }
}

TEST(SoftContacts, DeepCopy){
    {
        Model model(modelWithSoftContact);
        rigidbody::SoftContactNode& contact(model.softContact(0));
        rigidbody::SoftContactSphere &sphere(dynamic_cast<rigidbody::SoftContactSphere&>(contact));

        rigidbody::SoftContactSphere shallowCopy(sphere);
        rigidbody::SoftContactSphere deepCopyNow(sphere.DeepCopy());
        rigidbody::SoftContactSphere deepCopyLater;
        deepCopyLater.DeepCopy(sphere);

        {
            SCALAR_TO_DOUBLE(radiusTrue, sphere.radius());
            SCALAR_TO_DOUBLE(radiusShallow, shallowCopy.radius());
            SCALAR_TO_DOUBLE(radiusNow, deepCopyNow.radius());
            SCALAR_TO_DOUBLE(radiusLater, deepCopyLater.radius());

            SCALAR_TO_DOUBLE(stiffnessTrue, sphere.stiffness());
            SCALAR_TO_DOUBLE(stiffnessShallow, shallowCopy.stiffness());
            SCALAR_TO_DOUBLE(stiffnessNow, deepCopyNow.stiffness());
            SCALAR_TO_DOUBLE(stiffnessLater, deepCopyLater.stiffness());

            SCALAR_TO_DOUBLE(dampingTrue, sphere.damping());
            SCALAR_TO_DOUBLE(dampingShallow, shallowCopy.damping());
            SCALAR_TO_DOUBLE(dampingNow, deepCopyNow.damping());
            SCALAR_TO_DOUBLE(dampingLater, deepCopyLater.damping());

            SCALAR_TO_DOUBLE(muStaticTrue, sphere.muStatic());
            SCALAR_TO_DOUBLE(muStaticShallow, shallowCopy.muStatic());
            SCALAR_TO_DOUBLE(muStaticNow, deepCopyNow.muStatic());
            SCALAR_TO_DOUBLE(muStaticLater, deepCopyLater.muStatic());

            SCALAR_TO_DOUBLE(muDynamicTrue, sphere.muDynamic());
            SCALAR_TO_DOUBLE(muDynamicShallow, shallowCopy.muDynamic());
            SCALAR_TO_DOUBLE(muDynamicNow, deepCopyNow.muDynamic());
            SCALAR_TO_DOUBLE(muDynamicLater, deepCopyLater.muDynamic());

            SCALAR_TO_DOUBLE(muViscousTrue, sphere.muViscous());
            SCALAR_TO_DOUBLE(muViscousShallow, shallowCopy.muViscous());
            SCALAR_TO_DOUBLE(muViscousNow, deepCopyNow.muViscous());
            SCALAR_TO_DOUBLE(muViscousLater, deepCopyLater.muViscous());

            SCALAR_TO_DOUBLE(transitionVelocityTrue, sphere.transitionVelocity());
            SCALAR_TO_DOUBLE(transitionVelocityShallow, shallowCopy.transitionVelocity());
            SCALAR_TO_DOUBLE(transitionVelocityNow, deepCopyNow.transitionVelocity());
            SCALAR_TO_DOUBLE(transitionVelocityLater, deepCopyLater.transitionVelocity());

            EXPECT_NEAR(radiusTrue, 5., requiredPrecision);
            EXPECT_NEAR(radiusShallow, 5., requiredPrecision);
            EXPECT_NEAR(radiusNow, 5., requiredPrecision);
            EXPECT_NEAR(radiusLater, 5., requiredPrecision);

            EXPECT_NEAR(stiffnessTrue, 6., requiredPrecision);
            EXPECT_NEAR(stiffnessShallow, 6., requiredPrecision);
            EXPECT_NEAR(stiffnessNow, 6., requiredPrecision);
            EXPECT_NEAR(stiffnessLater, 6., requiredPrecision);

            EXPECT_NEAR(dampingTrue, 7., requiredPrecision);
            EXPECT_NEAR(dampingShallow, 7., requiredPrecision);
            EXPECT_NEAR(dampingNow, 7., requiredPrecision);
            EXPECT_NEAR(dampingLater, 7., requiredPrecision);

            EXPECT_NEAR(muStaticTrue, 0.8, requiredPrecision);
            EXPECT_NEAR(muStaticShallow, 0.8, requiredPrecision);
            EXPECT_NEAR(muStaticNow, 0.8, requiredPrecision);
            EXPECT_NEAR(muStaticLater, 0.8, requiredPrecision);

            EXPECT_NEAR(muDynamicTrue, 0.7, requiredPrecision);
            EXPECT_NEAR(muDynamicShallow, 0.7, requiredPrecision);
            EXPECT_NEAR(muDynamicNow, 0.7, requiredPrecision);
            EXPECT_NEAR(muDynamicLater, 0.7, requiredPrecision);

            EXPECT_NEAR(muViscousTrue, 0.5, requiredPrecision);
            EXPECT_NEAR(muViscousShallow, 0.5, requiredPrecision);
            EXPECT_NEAR(muViscousNow, 0.5, requiredPrecision);
            EXPECT_NEAR(muViscousLater, 0.5, requiredPrecision);

            EXPECT_NEAR(transitionVelocityTrue, 0.01, requiredPrecision);
            EXPECT_NEAR(transitionVelocityShallow, 0.01, requiredPrecision);
            EXPECT_NEAR(transitionVelocityNow, 0.01, requiredPrecision);
            EXPECT_NEAR(transitionVelocityLater, 0.01, requiredPrecision);
        }

        sphere.setRadius(100);
        sphere.setStiffness(101);
        sphere.setDamping(102);
        sphere.setMuStatic(103);
        sphere.setMuDynamic(104);
        sphere.setMuViscous(105);
        sphere.setTransitionVelocity(106);

        {
            SCALAR_TO_DOUBLE(radiusTrue, sphere.radius());
            SCALAR_TO_DOUBLE(radiusShallow, shallowCopy.radius());
            SCALAR_TO_DOUBLE(radiusNow, deepCopyNow.radius());
            SCALAR_TO_DOUBLE(radiusLater, deepCopyLater.radius());

            SCALAR_TO_DOUBLE(stiffnessTrue, sphere.stiffness());
            SCALAR_TO_DOUBLE(stiffnessShallow, shallowCopy.stiffness());
            SCALAR_TO_DOUBLE(stiffnessNow, deepCopyNow.stiffness());
            SCALAR_TO_DOUBLE(stiffnessLater, deepCopyLater.stiffness());

            SCALAR_TO_DOUBLE(dampingTrue, sphere.damping());
            SCALAR_TO_DOUBLE(dampingShallow, shallowCopy.damping());
            SCALAR_TO_DOUBLE(dampingNow, deepCopyNow.damping());
            SCALAR_TO_DOUBLE(dampingLater, deepCopyLater.damping());

            SCALAR_TO_DOUBLE(muStaticTrue, sphere.muStatic());
            SCALAR_TO_DOUBLE(muStaticShallow, shallowCopy.muStatic());
            SCALAR_TO_DOUBLE(muStaticNow, deepCopyNow.muStatic());
            SCALAR_TO_DOUBLE(muStaticLater, deepCopyLater.muStatic());

            SCALAR_TO_DOUBLE(muDynamicTrue, sphere.muDynamic());
            SCALAR_TO_DOUBLE(muDynamicShallow, shallowCopy.muDynamic());
            SCALAR_TO_DOUBLE(muDynamicNow, deepCopyNow.muDynamic());
            SCALAR_TO_DOUBLE(muDynamicLater, deepCopyLater.muDynamic());

            SCALAR_TO_DOUBLE(muViscousTrue, sphere.muViscous());
            SCALAR_TO_DOUBLE(muViscousShallow, shallowCopy.muViscous());
            SCALAR_TO_DOUBLE(muViscousNow, deepCopyNow.muViscous());
            SCALAR_TO_DOUBLE(muViscousLater, deepCopyLater.muViscous());

            SCALAR_TO_DOUBLE(transitionVelocityTrue, sphere.transitionVelocity());
            SCALAR_TO_DOUBLE(transitionVelocityShallow, shallowCopy.transitionVelocity());
            SCALAR_TO_DOUBLE(transitionVelocityNow, deepCopyNow.transitionVelocity());
            SCALAR_TO_DOUBLE(transitionVelocityLater, deepCopyLater.transitionVelocity());

            EXPECT_NEAR(radiusTrue, 100., requiredPrecision);
            EXPECT_NEAR(radiusShallow, 100., requiredPrecision);
            EXPECT_NEAR(radiusNow, 5., requiredPrecision);
            EXPECT_NEAR(radiusLater, 5., requiredPrecision);

            EXPECT_NEAR(stiffnessTrue, 101., requiredPrecision);
            EXPECT_NEAR(stiffnessShallow, 101., requiredPrecision);
            EXPECT_NEAR(stiffnessNow, 6., requiredPrecision);
            EXPECT_NEAR(stiffnessLater, 6., requiredPrecision);

            EXPECT_NEAR(dampingTrue, 102., requiredPrecision);
            EXPECT_NEAR(dampingShallow, 102., requiredPrecision);
            EXPECT_NEAR(dampingNow, 7., requiredPrecision);
            EXPECT_NEAR(dampingLater, 7., requiredPrecision);

            EXPECT_NEAR(muStaticTrue, 103, requiredPrecision);
            EXPECT_NEAR(muStaticShallow, 103, requiredPrecision);
            EXPECT_NEAR(muStaticNow, 0.8, requiredPrecision);
            EXPECT_NEAR(muStaticLater, 0.8, requiredPrecision);

            EXPECT_NEAR(muDynamicTrue, 104, requiredPrecision);
            EXPECT_NEAR(muDynamicShallow, 104, requiredPrecision);
            EXPECT_NEAR(muDynamicNow, 0.7, requiredPrecision);
            EXPECT_NEAR(muDynamicLater, 0.7, requiredPrecision);

            EXPECT_NEAR(muViscousTrue, 105, requiredPrecision);
            EXPECT_NEAR(muViscousShallow, 105, requiredPrecision);
            EXPECT_NEAR(muViscousNow, 0.5, requiredPrecision);
            EXPECT_NEAR(muViscousLater, 0.5, requiredPrecision);

            EXPECT_NEAR(transitionVelocityTrue, 106, requiredPrecision);
            EXPECT_NEAR(transitionVelocityShallow, 106, requiredPrecision);
            EXPECT_NEAR(transitionVelocityNow, 0.01, requiredPrecision);
            EXPECT_NEAR(transitionVelocityLater, 0.01, requiredPrecision);
        }
    }

}

TEST(SoftContacts, unitTest){
    rigidbody::SoftContactSphere sphere(0, 0, 0, 0.05, 1e6, 4, 0.8, 0.7, 0.5);
    {
        utils::Vector3d x(0.01, 0, 0.07);
        utils::Vector3d dx(0.01, 0, -0.01);
        utils::Vector3d angularVelocity(1, 2, 3);

        utils::Vector3d force = sphere.computeForce(x, dx, angularVelocity);

        std::vector<double> forceExpected = {0.003612873666450995, -0.0020071520369172192, 0.0054920962166388866};
        for (unsigned int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(f, force(i));
            EXPECT_NEAR(f, forceExpected[i], requiredPrecision);
        }
    }

    {
        utils::Vector3d x(0.01, 0, 0.0501);
        utils::Vector3d dx(0.01, 0, -0.01);
        utils::Vector3d angularVelocity(0, 0, 0);

        utils::Vector3d force = sphere.computeForce(x, dx, angularVelocity);

        std::vector<double> forceExpected = {-0.11760935061746022, 0., 0.15327642466721322};
        for (unsigned int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(f, force(i));
            EXPECT_NEAR(f, forceExpected[i], requiredPrecision);
        }
    }

    {
        utils::Vector3d x(0.01, 0, 0.049);
        utils::Vector3d dx(0.01, 0.01, -0.01);
        utils::Vector3d angularVelocity(0, 0, 0);

        utils::Vector3d force = sphere.computeForce(x, dx, angularVelocity);

        std::vector<double> forceExpected = {-3.5460071631036567, -3.5460071631036567, 6.4525442574502909};
        for (unsigned int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(f, force(i));
            EXPECT_NEAR(f, forceExpected[i], requiredPrecision);
        }
    }
}

TEST(SoftContacts, ForceAtOrigin){
    Model model(modelWithSoftContact);
    rigidbody::SoftContactSphere sphere(model.softContact(0));

    {
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity QDot(model);
        FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.}));
        FILL_VECTOR(QDot, std::vector<double>({0.1, 0.1, 0.1, 0.1}));

        utils::SpatialVector force = sphere.computeForceAtOrigin(model, Q, QDot);

        std::vector<double> forceExpected = {-670.95355043718416, 2.944729504204179, 2.2119497381886286,
                                             0, -221.1949738188676, 294.47295042042418};
        for (unsigned int i = 0; i < 6; ++i) {
            SCALAR_TO_DOUBLE(f, force(i));
            EXPECT_NEAR(f, forceExpected[i], requiredPrecision);
        }
    }
}

static std::vector<double> Qtest = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                     0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3
                                   };

TEST(GeneralizedCoordinates, unitTest)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::GeneralizedCoordinates Q(model);
        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            Q[i] = Qtest[i];
        }

        rigidbody::GeneralizedCoordinates newQ(Q);

        std::vector<double> Q_expected = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                           0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3
                                         };

        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            SCALAR_TO_DOUBLE(q, newQ[i]);
            EXPECT_NEAR(q, Q_expected[i], requiredPrecision);
        }
    }
}

TEST(GeneralizedVelocity, unitTest)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::GeneralizedVelocity Qdot(model);
        for (unsigned int i = 0; i < model.nbQdot(); ++i) {
            Qdot[i] = Qtest[i] * 10;
        }

        std::vector<double> Qdot_expected = { 1., 1., 1., 3., 3., 3.,
                                              3., 3., 3., 3., 3., 4., 3.
                                            };

        rigidbody::GeneralizedVelocity newQdot(Qdot);

        for (unsigned int i = 0; i < model.nbQdot(); ++i) {
            SCALAR_TO_DOUBLE(qdot, newQdot[i]);
            EXPECT_NEAR(qdot, Qdot_expected[i], requiredPrecision);
        }
    }
    {
        rigidbody::GeneralizedVelocity Qdot;
        SCALAR_TO_DOUBLE(qdotNorm, Qdot.norm());
        EXPECT_NEAR(qdotNorm, 0., requiredPrecision);

        rigidbody::GeneralizedVelocity newQdot(Qdot);
        SCALAR_TO_DOUBLE(newQdotNorm, newQdot.norm());
        EXPECT_NEAR(newQdotNorm, 0., requiredPrecision);
    }
}

TEST(GeneralizedAcceleration, unitTest)
{
    {
        rigidbody::GeneralizedAcceleration Qddot;
        SCALAR_TO_DOUBLE(QddotNorm, Qddot.norm());
        EXPECT_NEAR(QddotNorm, 0., requiredPrecision);
    }
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::GeneralizedAcceleration Qddot(model);
        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            Qddot[i] = Qtest[i] * 100;
        }

        std::vector<double> Qddot_expected = { 10., 10., 10., 30., 30., 30.,
                                               30., 30., 30., 30., 30., 40., 30.
                                             };

        rigidbody::GeneralizedAcceleration newQddot(Qddot);

        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            SCALAR_TO_DOUBLE(qddot, newQddot[i]);
            EXPECT_NEAR(qddot, Qddot_expected[i], requiredPrecision);
        }
    }
}

TEST(GeneralizedTorque, unitTest)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::GeneralizedTorque Tau(model);

#ifdef BIORBD_USE_EIGEN3_MATH
        Tau << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3;
#else
        Tau(0, 0) = 0.1;
        Tau(1, 0) = 0.1;
        Tau(2, 0) = 0.1;
        Tau(3, 0) = 0.3;
        Tau(4, 0) = 0.3;
        Tau(5, 0) = 0.3;
        Tau(6, 0) = 0.3;
        Tau(7, 0) = 0.3;
        Tau(8, 0) = 0.3;
        Tau(9, 0) = 0.3;
        Tau(10, 0) = 0.3;
        Tau(11, 0) = 0.4;
        Tau(12, 0) = 0.3;
#endif

        std::vector<double> Tau_expected = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                             0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3
                                           };

        for (unsigned int i = 0; i < 12; ++i) {
            SCALAR_TO_DOUBLE(tau, Tau[i]);
            EXPECT_NEAR(tau, Tau_expected[i], requiredPrecision);
        }

        rigidbody::GeneralizedTorque newTau(Tau);
        for (unsigned int i = 0; i < 12; ++i) {
            SCALAR_TO_DOUBLE(tau, newTau[i]);
            EXPECT_NEAR(tau, Tau_expected[i], requiredPrecision);
        }

        rigidbody::GeneralizedTorque tauWithNoArgument;
        SCALAR_TO_DOUBLE(tauWithNoArgumentNorm, tauWithNoArgument.norm());
        EXPECT_NEAR(tauWithNoArgumentNorm, 0., requiredPrecision);
    }
}

TEST(IMU, unitTest)
{
    rigidbody::IMU imu(true, false);
    EXPECT_EQ(imu.isTechnical(), true);
    EXPECT_EQ(imu.isAnatomical(), false);
}

TEST(IMU, DeepCopy)
{
    rigidbody::IMU imu(false, true);

    rigidbody::IMU shallowCopy(imu);
    rigidbody::IMU deepCopyNow(imu.DeepCopy());
    rigidbody::IMU deepCopyLater;
    deepCopyLater.DeepCopy(imu);

    EXPECT_EQ(shallowCopy.isTechnical(), false);
    EXPECT_EQ(deepCopyNow.isTechnical(), false);
    EXPECT_EQ(deepCopyLater.isTechnical(), false);
}

static std::string
modelPathForPyomecaman_withIMUs("models/IMUandCustomRT/pyomecaman_withIMUs.bioMod");
TEST(IMUs, unitTest)
{
    {
        rigidbody::IMUs imus;
        imus.addIMU(true, true);

        EXPECT_EQ(imus.nbIMUs(), 1);
        EXPECT_EQ(imus.IMU(0).isTechnical(), true);
    }
    {
        rigidbody::IMUs imus;
        rigidbody::IMU imu(true, false);
        imu.setName("imuName");
        imus.addIMU(imu);

        EXPECT_STREQ(imus.technicalIMU()[0].utils::Node::name().c_str(),
                     "imuName");
    }
    {
        Model model(modelPathForPyomecaman_withIMUs);
        rigidbody::IMUs imus(model);

        EXPECT_EQ(imus.anatomicalIMU().size(), 2);
        EXPECT_EQ(imus.technicalIMU().size(), 4);
    }
}

TEST(IMUs, deepCopy)
{
    Model model(modelPathForPyomecaman_withIMUs);
    rigidbody::IMUs imus(model);

    rigidbody::IMUs shallowCopy(imus);
    rigidbody::IMUs deepCopyNow(imus.DeepCopy());
    rigidbody::IMUs deepCopyLater;
    deepCopyLater.DeepCopy(imus);

    EXPECT_EQ(shallowCopy.nbIMUs(), 4);
    EXPECT_EQ(deepCopyNow.nbIMUs(), 4);
    EXPECT_EQ(deepCopyLater.nbIMUs(), 4);

    imus.addIMU(true, true);
    EXPECT_EQ(imus.nbIMUs(), 5);
    EXPECT_EQ(shallowCopy.nbIMUs(), 5);
    EXPECT_EQ(deepCopyNow.nbIMUs(), 4);
    EXPECT_EQ(deepCopyLater.nbIMUs(), 4);
}

TEST(Joints, copy)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Joints joints(model);

        rigidbody::Joints shallowCopy(joints);
        rigidbody::Joints deepCopyNow(joints.DeepCopy());
        rigidbody::Joints deepCopyLater;
        deepCopyLater.DeepCopy(joints);
        {
            SCALAR_TO_DOUBLE(shallowCopyMass, shallowCopy.mass());
            SCALAR_TO_DOUBLE(deepCopyNowMass, deepCopyNow.mass());
            SCALAR_TO_DOUBLE(deepCopyLaterMass, deepCopyLater.mass());
            EXPECT_NEAR(shallowCopyMass, 52.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyNowMass, 52.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterMass, 52.412120000000002, requiredPrecision);
        }

        rigidbody::SegmentCharacteristics characteristics(
            10, utils::Vector3d(0.5, 0.5, 0.5),
            utils::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
        std::vector<utils::Range> ranges(6);

        joints.AddSegment("segmentName", "parentName", "zyx", "yzx", ranges, ranges,
                          ranges,
                          characteristics, RigidBodyDynamics::Math::SpatialTransform());

        {
            SCALAR_TO_DOUBLE(jointsMass, joints.mass());
            SCALAR_TO_DOUBLE(shallowCopyMass, shallowCopy.mass());
            SCALAR_TO_DOUBLE(deepCopyNowMass, deepCopyNow.mass());
            SCALAR_TO_DOUBLE(deepCopyLaterMass, deepCopyLater.mass());
            EXPECT_NEAR(jointsMass, 62.412120000000002, requiredPrecision);
            EXPECT_NEAR(shallowCopyMass, 62.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyNowMass, 52.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterMass, 52.412120000000002, requiredPrecision);
        }
    }
}

TEST(Joints, unitTest)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Joints joints(model);
        std::vector <utils::String> names(joints.nameDof());

        std::vector<utils::String> expectedNames(joints.nbDof());
        expectedNames = { "Pelvis_TransY", "Pelvis_TransZ", "Pelvis_RotX",
                          "BrasD_RotZ", "BrasD_RotX", "BrasG_RotZ", "BrasG_RotX",
                          "CuisseD_RotX", "JambeD_RotX", "PiedD_RotX", "CuisseG_RotX",
                          "JambeG_RotX", "PiedG_RotX"
                        };


        for (unsigned int i = 0; i < joints.nbDof(); ++i) {
            EXPECT_STREQ(names[i].c_str(), expectedNames[i].c_str());
        }
    }
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Joints joints(model);
        rigidbody::Segment segmentToTest(joints.segment("Tronc"));

        EXPECT_EQ(segmentToTest.id(), INT_MAX); //TODO: Verify ID value
    }
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Joints joints(model);
        rigidbody::GeneralizedCoordinates Q(model);
        rigidbody::GeneralizedVelocity Qdot(model);
        for (size_t i=0; i<model.nbQ(); ++i) {
            Q[i] = static_cast<double>(i) * 0.2;
            Qdot[i] = static_cast<double>(i) * 1.2;
        }

        utils::Vector3d angularMomentum(joints.angularMomentum(Q, Qdot));
        std::vector<double> expectedAngularMomentum = {15.957205552043206, -2.399856350425782, 2.0751269909741334};
        
        for (int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(momentum, angularMomentum[i]);
            EXPECT_NEAR(momentum, expectedAngularMomentum[i], requiredPrecision);
        }
        
        utils::Vector3d bodyAngularVelocity(joints.bodyAngularVelocity(Q, Qdot));
        std::vector<double> expectedBodyAngularVelocity = {5.4094513140292122, -0.73173953080349363, 0.73450575671796559};
        
        for (int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(velocity, bodyAngularVelocity[i]);
            EXPECT_NEAR(velocity, expectedBodyAngularVelocity[i], requiredPrecision);
        }
    }
    {
        Model model(modelPathForGeneralTesting);
        EXPECT_EQ(model.getBodyRbdlIdToBiorbdId(3),0);
        EXPECT_EQ(model.getBodyRbdlIdToBiorbdId(4),-1);
    }
    {
        Model model(modelPathForGeneralTesting);
        EXPECT_EQ(model.getBodyBiorbdIdToRbdlId(0),3);
        EXPECT_EQ(model.getBodyBiorbdIdToRbdlId(1),INT_MAX);
        EXPECT_EQ(model.getBodyBiorbdIdToRbdlId(10),13);
    }

}


TEST(Joints, combineExtForceAndSoftContact)
{
    {
        // Only f_contacts
        Model model(modelPathForGeneralTesting);

        rigidbody::GeneralizedCoordinates Q(model);
        FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2, 0.3,
                                           -2.01, -3.01, -3.01, 0.1, 0.2, 0.3, 0.4}));

        rigidbody::GeneralizedVelocity QDot(model);
        FILL_VECTOR(QDot, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2, 0.3,
                                           -2.01, -3.01, -3.01, 0.1, 0.2, 0.3, 0.4}));

        std::vector<utils::SpatialVector> *allForces_ext;
        allForces_ext = nullptr;

        std::vector<utils::Vector> realAllForces;
        std::vector<utils::Vector> *allForces;
        allForces = &realAllForces;

        utils::Vector force1;
        force1 = utils::Vector(2);
        force1[0] = 1;
        force1[1] = 2;

        utils::Vector force2;
        force2 = utils::Vector(1);
        force2[0] = 3;

        utils::Vector force3;
        force3 = utils::Vector(2);
        force3[0] = 4;
        force3[1] = 5;

        utils::Vector force4;
        force4 = utils::Vector(1);
        force4[0] = 6;

        allForces->push_back(force1);
        allForces->push_back(force2);
        allForces->push_back(force3);
        allForces->push_back(force4);

        WrapperToJointsContactSoftcontact wrappedModel(model);
        std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl(
                    wrappedModel.wrapCombineExtForceAndSoftContact(allForces_ext, allForces, Q, QDot, true)
                                                            );

        RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
        RigidBodyDynamics::Math::SpatialVector sp_dof12(-7.592268755852077, -0.7864099999999999, 0.14995, 0, 1.0, 5.0);
        RigidBodyDynamics::Math::SpatialVector sp_dof15(-17.952947566495585, 1.72277, -0.5998, 0.0, 4.0, 11.0);

        std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

        sp_expected.push_back(sp_zero); // Dof 0
        sp_expected.push_back(sp_zero); // Dof 1
        sp_expected.push_back(sp_zero); // Dof 2
        sp_expected.push_back(sp_zero); // Dof 3
        sp_expected.push_back(sp_zero); // Dof 4
        sp_expected.push_back(sp_zero); // Dof 5
        sp_expected.push_back(sp_zero); // Dof 6
        sp_expected.push_back(sp_zero); // Dof 7
        sp_expected.push_back(sp_zero); // Dof 8
        sp_expected.push_back(sp_zero); // Dof 9
        sp_expected.push_back(sp_zero); // Dof 10
        sp_expected.push_back(sp_zero); // Dof 11
        sp_expected.push_back(sp_dof12); // Dof 12
        sp_expected.push_back(sp_zero); // Dof 13
        sp_expected.push_back(sp_zero); // Dof 14
        sp_expected.push_back(sp_dof15); // Dof 15

        for (unsigned int i = 0; i < 16; ++i){

            for (unsigned int j = 0; j < 6; ++j)
                    {
                        SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                        SCALAR_TO_DOUBLE(f, (*f_ext_rbdl)[i](j));
                        EXPECT_NEAR(f, f_expected, requiredPrecision);
                    }
        }
    }
    {
    // Only soft_contacts
    Model model(modelWithSoftContact);

    rigidbody::GeneralizedCoordinates Q(model);
    FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    rigidbody::GeneralizedVelocity QDot(model);
    FILL_VECTOR(QDot, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    std::vector<utils::SpatialVector> *allForces_ext;
    allForces_ext = nullptr;

    WrapperToJointsContactSoftcontact wrappedModel(model);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl(
                wrappedModel.wrapCombineExtForceAndSoftContact(allForces_ext, nullptr, Q, QDot, true)
                                                        );

    RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
    RigidBodyDynamics::Math::SpatialVector sp_dof4(185392.9862903644, -249642.95301694548, 238700.3791127471, 74247.2670562476, 102146.62960989607, 49317.07505542255);

    std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

    sp_expected.push_back(sp_zero); // Dof 0
    sp_expected.push_back(sp_zero); // Dof 1
    sp_expected.push_back(sp_zero); // Dof 2
    sp_expected.push_back(sp_zero); // Dof 3
    sp_expected.push_back(sp_dof4); // Dof 4

    for (unsigned int i = 0; i < 5; ++i){

        for (unsigned int j = 0; j < 6; ++j)
                {
                    SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                    SCALAR_TO_DOUBLE(f, (*f_ext_rbdl)[i](j));
                    EXPECT_NEAR(f, f_expected, 1e-9);
                }
        }
    }
    {
    // Only external forces
    Model model(modelWithRigidContactsExternalForces);

    rigidbody::GeneralizedCoordinates Q(model);
    FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    rigidbody::GeneralizedVelocity QDot(model);
    FILL_VECTOR(QDot, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    std::vector<utils::SpatialVector> realallForces_ext;
    std::vector<utils::SpatialVector> *allForces_ext;
    allForces_ext = &realallForces_ext;

    utils::SpatialVector force1(1, 2, 3, 4, 5, 6);
    allForces_ext->push_back(force1);

    WrapperToJointsContactSoftcontact wrappedModel(model);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl(
                wrappedModel.wrapCombineExtForceAndSoftContact(allForces_ext, nullptr, Q, QDot, true)
                                                        );

    RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
    RigidBodyDynamics::Math::SpatialVector sp_dof4(1, 2, 3, 4, 5, 6);

    std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

    sp_expected.push_back(sp_zero); // Dof 0
    sp_expected.push_back(sp_zero); // Dof 1
    sp_expected.push_back(sp_zero); // Dof 2
    sp_expected.push_back(sp_zero); // Dof 3
    sp_expected.push_back(sp_dof4); // Dof 4

    for (unsigned int i = 0; i < 5; ++i){

        for (unsigned int j = 0; j < 6; ++j)
                {
                    SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                    SCALAR_TO_DOUBLE(f, (*f_ext_rbdl)[i](j));
                    EXPECT_NEAR(f, f_expected, requiredPrecision);
                }
    }
    }
    {
    // external forces + rigid contacts
    Model model(modelWithRigidContactsExternalForces);

    rigidbody::GeneralizedCoordinates Q(model);
    FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    rigidbody::GeneralizedVelocity QDot(model);
    FILL_VECTOR(QDot, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    std::vector<utils::SpatialVector> realallForces_ext;
    std::vector<utils::SpatialVector> *allForces_ext;
    allForces_ext = &realallForces_ext;

    utils::SpatialVector force(1, 2, 3, 4, 5, 6);
    allForces_ext->push_back(force);

    std::vector<utils::Vector> realAllForcesRigidContact;
    std::vector<utils::Vector> *allForcesRigidContact;
    allForcesRigidContact = &realAllForcesRigidContact;

    utils::Vector force1;
    force1 = utils::Vector(2);
    force1[0] = 1;
    force1[1] = 2;

    utils::Vector force2;
    force2 = utils::Vector(1);
    force2[0] = 3;

    allForcesRigidContact->push_back(force1);
    allForcesRigidContact->push_back(force2);

    WrapperToJointsContactSoftcontact wrappedModel(model);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl(
                wrappedModel.wrapCombineExtForceAndSoftContact(allForces_ext, allForcesRigidContact, Q, QDot, true)
                                                        );

    RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
    RigidBodyDynamics::Math::SpatialVector sp_dof4(-10.694693700837254, 12.040834024296124, 0.98598321280716972, 4, 6, 11);

    std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

    sp_expected.push_back(sp_zero); // Dof 0
    sp_expected.push_back(sp_zero); // Dof 1
    sp_expected.push_back(sp_zero); // Dof 2
    sp_expected.push_back(sp_zero); // Dof 3
    sp_expected.push_back(sp_dof4); // Dof 4

    for (unsigned int i = 0; i < 5; ++i){

        for (unsigned int j = 0; j < 6; ++j)
                {
                    SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                    SCALAR_TO_DOUBLE(f, (*f_ext_rbdl)[i](j));
                    EXPECT_NEAR(f, f_expected, requiredPrecision);
                }
    }
    }
    {
    // SoftContact + rigid contacts
    Model model(modelWithSoftContactRigidContactsExternalForces);

    rigidbody::GeneralizedCoordinates Q(model);
    FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    rigidbody::GeneralizedVelocity QDot(model);
    FILL_VECTOR(QDot, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    std::vector<utils::Vector> realAllForcesRigidContact;
    std::vector<utils::Vector> *allForcesRigidContact;
    allForcesRigidContact = &realAllForcesRigidContact;

    utils::Vector force1;
    force1 = utils::Vector(2);
    force1[0] = 1;
    force1[1] = 2;

    utils::Vector force2;
    force2 = utils::Vector(1);
    force2[0] = 3;

    allForcesRigidContact->push_back(force1);
    allForcesRigidContact->push_back(force2);

    WrapperToJointsContactSoftcontact wrappedModel(model);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl(
                wrappedModel.wrapCombineExtForceAndSoftContact(nullptr, allForcesRigidContact, Q, QDot, true)
                                                        );

    RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
    RigidBodyDynamics::Math::SpatialVector sp_dof4(185381.29159666356, -249632.91218292119, 238698.36509595989, 74247.267056247598, 102147.62960989607, 49322.075055422552);

    std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

    sp_expected.push_back(sp_zero); // Dof 0
    sp_expected.push_back(sp_zero); // Dof 1
    sp_expected.push_back(sp_zero); // Dof 2
    sp_expected.push_back(sp_zero); // Dof 3
    sp_expected.push_back(sp_dof4); // Dof 4

    for (unsigned int i = 0; i < 5; ++i){

        for (unsigned int j = 0; j < 6; ++j)
                {
                    SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                    SCALAR_TO_DOUBLE(f, (*f_ext_rbdl)[i](j));
                    EXPECT_NEAR(f, f_expected, 1e-9);
                }
    }
    }
    {
    // SoftContact + external forces
    Model model(modelWithSoftContactRigidContactsExternalForces);

    rigidbody::GeneralizedCoordinates Q(model);
    FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    rigidbody::GeneralizedVelocity QDot(model);
    FILL_VECTOR(QDot, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    std::vector<utils::SpatialVector> realallForces_ext;
    std::vector<utils::SpatialVector> *allForces_ext;
    allForces_ext = &realallForces_ext;

    utils::SpatialVector force(1, 2, 3, 4, 5, 6);
    allForces_ext->push_back(force);

    WrapperToJointsContactSoftcontact wrappedModel(model);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl(
                wrappedModel.wrapCombineExtForceAndSoftContact(allForces_ext, nullptr, Q, QDot, true)
                                                        );

    RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
    RigidBodyDynamics::Math::SpatialVector sp_dof4(185393.98629036441, -249640.95301694548, 238703.37911274709, 74251.267056247598, 102151.62960989607, 49323.075055422552);

    std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

    sp_expected.push_back(sp_zero); // Dof 0
    sp_expected.push_back(sp_zero); // Dof 1
    sp_expected.push_back(sp_zero); // Dof 2
    sp_expected.push_back(sp_zero); // Dof 3
    sp_expected.push_back(sp_dof4); // Dof 4

    for (unsigned int i = 0; i < 5; ++i){

        for (unsigned int j = 0; j < 6; ++j)
                {
                    SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                    SCALAR_TO_DOUBLE(f, (*f_ext_rbdl)[i](j));
                    EXPECT_NEAR(f, f_expected, 1e-9);
                }
    }
    }
    {
    // SoftContact + external forces + rigid contacts
    Model model(modelWithSoftContactRigidContactsExternalForces);

    rigidbody::GeneralizedCoordinates Q(model);
    FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    rigidbody::GeneralizedVelocity QDot(model);
    FILL_VECTOR(QDot, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2}));

    std::vector<utils::SpatialVector> realallForces_ext;
    std::vector<utils::SpatialVector> *allForces_ext;
    allForces_ext = &realallForces_ext;

    utils::SpatialVector force(1, 2, 3, 4, 5, 6);
    allForces_ext->push_back(force);

    std::vector<utils::Vector> realAllForcesRigidContact;
    std::vector<utils::Vector> *allForcesRigidContact;
    allForcesRigidContact = &realAllForcesRigidContact;

    utils::Vector force1;
    force1 = utils::Vector(2);
    force1[0] = 1;
    force1[1] = 2;

    utils::Vector force2;
    force2 = utils::Vector(1);
    force2[0] = 3;

    allForcesRigidContact->push_back(force1);
    allForcesRigidContact->push_back(force2);

    WrapperToJointsContactSoftcontact wrappedModel(model);
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext_rbdl(
                wrappedModel.wrapCombineExtForceAndSoftContact(allForces_ext, allForcesRigidContact, Q, QDot, true)
                                                        );

    RigidBodyDynamics::Math::SpatialVector sp_zero(0, 0, 0, 0, 0, 0);
    RigidBodyDynamics::Math::SpatialVector sp_dof4(185382.29159666356, -249630.91218292119, 238701.36509595989, 74251.267056247598, 102152.62960989607, 49328.075055422552);

    std::vector<RigidBodyDynamics::Math::SpatialVector> sp_expected;

    sp_expected.push_back(sp_zero); // Dof 0
    sp_expected.push_back(sp_zero); // Dof 1
    sp_expected.push_back(sp_zero); // Dof 2
    sp_expected.push_back(sp_zero); // Dof 3
    sp_expected.push_back(sp_dof4); // Dof 4

    for (unsigned int i = 0; i < 5; ++i){

        for (unsigned int j = 0; j < 6; ++j)
                {
                    SCALAR_TO_DOUBLE(f_expected, sp_expected[i](j));
                    SCALAR_TO_DOUBLE(f, (*f_ext_rbdl)[i](j));
                    EXPECT_NEAR(f, f_expected, 1e-9);
                }
    }
    }
}

TEST(Joints, massMatrixInverse)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::GeneralizedCoordinates Q(model);
        FILL_VECTOR(Q, std::vector<double>({-2.01, -3.01, -3.01, 0.1, 0.2, 0.3,
                                           -2.01, -3.01, -3.01, 0.1, 0.2, 0.3, 0.4}));

        utils::Matrix M(model.massMatrix(Q));
        utils::Matrix Minv_num = M.inverse();

        utils::Matrix Minv_symbolic = model.massMatrixInverse(Q);

        for (unsigned int j = 0; j < model.dof_count; j++)
        {
            for (unsigned int i = 0; i < model.dof_count; i++)
            {
                SCALAR_TO_DOUBLE(Minv_num_ij, Minv_num(i,j));
                SCALAR_TO_DOUBLE(Minv_symbolic_ij, Minv_symbolic(i,j));
                EXPECT_NEAR(Minv_num_ij, Minv_symbolic_ij, requiredPrecision);
            }
        }
    }
}

TEST(Markers, copy)
{
    {
        Model model(modelPathForGeneralTesting);
        rigidbody::Markers markers(model);

        rigidbody::Markers shallowCopy(markers);
        rigidbody::Markers deepCopyNow(markers.DeepCopy());
        rigidbody::Markers deepCopyLater;
        deepCopyLater.DeepCopy(markers);

        EXPECT_EQ(markers.nbMarkers(), 97);
        EXPECT_EQ(shallowCopy.nbMarkers(), 97);
        EXPECT_EQ(deepCopyNow.nbMarkers(), 97);
        EXPECT_EQ(deepCopyLater.nbMarkers(), 97);


        rigidbody::NodeSegment nodeSegment;
        markers.addMarker(nodeSegment,
                          "markerName", "parentName", true, true, "x", 98);

        EXPECT_EQ(markers.nbMarkers(), 98);
        EXPECT_EQ(shallowCopy.nbMarkers(), 98);
        EXPECT_EQ(deepCopyNow.nbMarkers(), 97);
        EXPECT_EQ(deepCopyLater.nbMarkers(), 97);
    }
}

TEST(SegmentCharacteristics, length)
{
    rigidbody::SegmentCharacteristics segmentCharac;
    SCALAR_TO_DOUBLE(length1, segmentCharac.length());
    EXPECT_NEAR(length1, 0., requiredPrecision);
    segmentCharac.setLength(2.);
    SCALAR_TO_DOUBLE(length2, segmentCharac.length());
    EXPECT_NEAR(length2, 2., requiredPrecision);
}

TEST(Segment, nameDof)
{
    Model model(modelPathForGeneralTesting);
    EXPECT_THROW(model.segment(128), std::runtime_error);
}

static std::string modelPathForRTsane("models/IMUandCustomRT/RT_sane.bioMod");
static std::string
modelPathForRTwrong1("models/IMUandCustomRT/RT_wrong1.bioMod");
static std::string
modelPathForRTwrong2("models/IMUandCustomRT/RT_wrong2.bioMod");
static std::string
modelPathForRTwrong3("models/IMUandCustomRT/RT_wrong3.bioMod");
static std::string
modelPathForRTwrong4("models/IMUandCustomRT/RT_wrong4.bioMod");
static std::string
modelPathForRTwrong5("models/IMUandCustomRT/RT_wrong5.bioMod");
static std::string
modelPathForRTwrong6("models/IMUandCustomRT/RT_wrong6.bioMod");
TEST(RotoTransNode, Read)
{
    {
        Model model(modelPathForRTsane);
        rigidbody::RotoTransNodes rt(model);

        EXPECT_EQ(rt.size(), 3);
    }

    {
        EXPECT_THROW(Model model(modelPathForRTwrong1), std::runtime_error);
        EXPECT_THROW(Model model(modelPathForRTwrong2), std::runtime_error);
        EXPECT_THROW(Model model(modelPathForRTwrong3), std::runtime_error);
        EXPECT_THROW(Model model(modelPathForRTwrong4), std::runtime_error);
        EXPECT_THROW(Model model(modelPathForRTwrong5), std::runtime_error);
        EXPECT_THROW(Model model(modelPathForRTwrong6), std::runtime_error);
    }
}

TEST(RotoTransNode, copy)
{
    Model model(modelPathForGeneralTesting);
    rigidbody::RotoTransNodes rtNode(model);
    utils::RotoTransNode rt(
        utils::RotoTrans(utils::Vector3d(2, 3, 4),
                                 utils::Vector3d(), "xyz"), "", "" );
    rtNode.addRT(rt);

    rigidbody::RotoTransNodes shallowCopy(rtNode);
    rigidbody::RotoTransNodes deepCopyNow(rtNode.DeepCopy());
    rigidbody::RotoTransNodes deepCopyLater;
    deepCopyLater.DeepCopy(rtNode);

    EXPECT_EQ(shallowCopy.RTs().size(), 1);
    EXPECT_EQ(deepCopyNow.RTs().size(), 1);
    EXPECT_EQ(deepCopyLater.RTs().size(), 1);

    rtNode.addRT(rt);
    EXPECT_EQ(rtNode.RTs().size(), 2);
    EXPECT_EQ(shallowCopy.RTs().size(), 2);
    EXPECT_EQ(deepCopyNow.RTs().size(), 1);
    EXPECT_EQ(deepCopyLater.RTs().size(), 1);
}

TEST(RotoTransNode, unitTest)
{
    {
        rigidbody::RotoTransNodes rtNode;
        utils::RotoTransNode rt(
            utils::RotoTrans(utils::Vector3d(2, 3, 4),
                                     utils::Vector3d(), "xyz"), "", "" );
        rtNode.addRT(rt);
        auto rt_vector(rtNode.RTs());
#ifndef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(rt_vector[0].norm(), 1.9999999999999998, requiredPrecision);
        EXPECT_NEAR(rtNode.RT(0).norm(), 1.9999999999999998, requiredPrecision);
#endif
    }
    {
        rigidbody::RotoTransNodes rtNode;
        utils::RotoTransNode rt(
            utils::RotoTrans(utils::Vector3d(2, 3, 4),
                                     utils::Vector3d(), "xyz"), "", "" );
        rtNode.addRT(rt);
        std::vector<utils::RotoTransNode> rt_vector(rtNode.RTs());
        rt_vector[0].setParent("parentName");
        rt_vector[0].setName("nameSet");
        std::vector<utils::String> expectedNames = { "nameSet" };
        for (unsigned int i = 0; i < rt_vector.size(); ++i) {
            EXPECT_STREQ(rtNode.RTs("parentName")[i].utils::Node::name().c_str(),
                         expectedNames[i].c_str());
            EXPECT_STREQ(rtNode.RTsNames()[i].c_str(), expectedNames[i].c_str());
        }
    }
    {
        rigidbody::RotoTransNodes rtNode;
        rtNode.addRT();
        unsigned int numberOfRTs(rtNode.nbRTs());
        EXPECT_EQ(numberOfRTs, 1);
    }
}


TEST(NodeSegment, unitTests)
{
    {
        rigidbody::NodeSegment nodeSegment(1.1, 2.2, 3.3);
        SCALAR_TO_DOUBLE(x, nodeSegment.x());
        SCALAR_TO_DOUBLE(y, nodeSegment.y());
        SCALAR_TO_DOUBLE(z, nodeSegment.z());
        EXPECT_NEAR(x, 1.1, requiredPrecision);
        EXPECT_NEAR(y, 2.2, requiredPrecision);
        EXPECT_NEAR(z, 3.3, requiredPrecision);
    }
    {
        rigidbody::NodeSegment nodeSegment(utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        EXPECT_STREQ(nodeSegment.parent().c_str(), "parentName");
    }
    {
        rigidbody::NodeSegment nodeSegment(utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);

        EXPECT_EQ(nodeSegment.isAxisKept(2), false);
        EXPECT_EQ(nodeSegment.isAxisRemoved(2), true);
    }
    {
        rigidbody::NodeSegment nodeSegment(utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        std::vector<utils::String> vector = { "x", "y" };
        nodeSegment.addAxesToRemove(vector);
        EXPECT_STREQ(nodeSegment.axesToRemove().c_str(), "xyz");
    }
    {
        rigidbody::NodeSegment nodeSegment(utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        std::vector<unsigned int> vector = {0, 1};
        nodeSegment.addAxesToRemove(vector);
        EXPECT_STREQ(nodeSegment.axesToRemove().c_str(), "xyz");
    }
    {
        rigidbody::NodeSegment nodeSegment(utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        EXPECT_THROW(nodeSegment.addAxesToRemove(4), std::runtime_error);
        utils::String string("m");
        EXPECT_THROW(nodeSegment.addAxesToRemove(string), std::runtime_error);
    }
}


TEST(NodeSegment, copy)
{
    rigidbody::NodeSegment nodeSegment(utils::Vector3d(2, 3, 4),
            "nodeSegmentName", "parentName", true, true, "z", 8);

    rigidbody::NodeSegment deepCopyNow(nodeSegment.DeepCopy());
    rigidbody::NodeSegment deepCopyLater;
    deepCopyLater.DeepCopy(nodeSegment);

    EXPECT_EQ(nodeSegment.nbAxesToRemove(), 1);
    EXPECT_EQ(deepCopyNow.nbAxesToRemove(), 1);
    EXPECT_EQ(deepCopyLater.nbAxesToRemove(), 1);
}

TEST(DegreesOfFreedom, count)
{
    {
        Model model(modelPathForGeneralTesting);
        EXPECT_EQ(model.nbQ(), 13);
        EXPECT_EQ(model.nbQdot(), 13);
        EXPECT_EQ(model.nbQddot(), 13);
        EXPECT_EQ(model.nbGeneralizedTorque(), 13);
        EXPECT_EQ(model.nbRoot(), 3);
    }
    {
        Model model(modelNoRoot);
        EXPECT_EQ(model.nbQ(), 13);
        EXPECT_EQ(model.nbQdot(), 13);
        EXPECT_EQ(model.nbQddot(), 13);
        EXPECT_EQ(model.nbGeneralizedTorque(), 13);
        EXPECT_EQ(model.nbRoot(), 3);
    }
}

TEST(DegressOfFreedom, ranges)
{
    Model model(modelPathForGeneralTesting);
    std::vector<utils::Range> QRanges;
    std::vector<utils::Range> QDotRanges;
    std::vector<utils::Range> QDDotRanges;

    // Pelvis
    QRanges = model.segment(0).QRanges();
    EXPECT_EQ(QRanges[0].min(), -15);
    EXPECT_EQ(QRanges[0].max(), 15);
    EXPECT_EQ(QRanges[1].min(), -15);
    EXPECT_EQ(QRanges[1].max(), 15);
    EXPECT_EQ(QRanges[2].min(), -M_PI+1);
    EXPECT_EQ(QRanges[2].max(), M_PI+1);

    QDotRanges = model.segment(0).QDotRanges();
    EXPECT_EQ(QDotRanges[0].min(), -150);
    EXPECT_EQ(QDotRanges[0].max(), 150);
    EXPECT_EQ(QDotRanges[1].min(), -150);
    EXPECT_EQ(QDotRanges[1].max(), 150);
    EXPECT_EQ(QDotRanges[2].min(), -(M_PI+1)*10);
    EXPECT_EQ(QDotRanges[2].max(), (M_PI+1)*10);

    QDDotRanges = model.segment(0).QDDotRanges();
    EXPECT_EQ(QDDotRanges[0].min(), -1500);
    EXPECT_EQ(QDDotRanges[0].max(), 1500);
    EXPECT_EQ(QDDotRanges[1].min(), -1500);
    EXPECT_EQ(QDDotRanges[1].max(), 1500);
    EXPECT_EQ(QDDotRanges[2].min(), -(M_PI+1)*100);
    EXPECT_EQ(QDDotRanges[2].max(), (M_PI+1)*100);

    // BrasD
    QRanges = model.segment(3).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI);
    EXPECT_EQ(QRanges[0].max(), M_PI);
    EXPECT_EQ(QRanges[1].min(), 0);
    EXPECT_EQ(QRanges[1].max(), M_PI);

    QDotRanges = model.segment(3).QDotRanges();
    EXPECT_EQ(QDotRanges[0].min(), -M_PI*10);
    EXPECT_EQ(QDotRanges[0].max(), M_PI*10);
    EXPECT_EQ(QDotRanges[1].min(), -M_PI*10);
    EXPECT_EQ(QDotRanges[1].max(), M_PI*10);

    QDDotRanges = model.segment(3).QDDotRanges();
    EXPECT_EQ(QDDotRanges[0].min(), -M_PI*100);
    EXPECT_EQ(QDDotRanges[0].max(), M_PI*100);
    EXPECT_EQ(QDDotRanges[1].min(), -M_PI*100);
    EXPECT_EQ(QDDotRanges[1].max(), M_PI*100);

    // BrasG
    QRanges = model.segment(4).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI);
    EXPECT_EQ(QRanges[0].max(), M_PI);
    EXPECT_EQ(QRanges[1].min(), 0);
    EXPECT_EQ(QRanges[1].max(), M_PI);

    // CuisseD
    QRanges = model.segment(5).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/12);
    EXPECT_EQ(QRanges[0].max(), M_PI/2+M_PI/3);

    // JambeD
    QRanges = model.segment(6).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2-M_PI/6);
    EXPECT_EQ(QRanges[0].max(), 0);

    // PiedD
    QRanges = model.segment(7).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2);
    EXPECT_EQ(QRanges[0].max(), M_PI/2);

    // CuisseG
    QRanges = model.segment(8).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/12);
    EXPECT_EQ(QRanges[0].max(), M_PI/2+M_PI/3);

    // JambeG
    QRanges = model.segment(9).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2-M_PI/6);
    EXPECT_EQ(QRanges[0].max(), 0);

    // PiedG
    QRanges = model.segment(10).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2);
    EXPECT_EQ(QRanges[0].max(), M_PI/2);
}

static std::vector<double> QtestPyomecaman = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                              0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3
                                             };
static std::vector<double> QtestEqualsMarker = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3};

TEST(CoM, kinematics)
{
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(Qdot, model);
    DECLARE_GENERALIZED_ACCELERATION(Qddot, model);

    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Q(i, 0) = QtestPyomecaman[i];
        Qdot(i, 0) = QtestPyomecaman[i]*10;
        Qddot(i, 0) = QtestPyomecaman[i]*100;
    }

    CALL_BIORBD_FUNCTION_1ARG(com, model, CoM, Q);
    CALL_BIORBD_FUNCTION_2ARGS(comDot, model, CoMdot, Q, Qdot);
    CALL_BIORBD_FUNCTION_3ARGS(comDdot, model, CoMddot, Q, Qdot, Qddot);

    std::vector<double> expectedCom = {-0.0034679564024098523, 0.15680579877453169, 0.07808112642459612};
    std::vector<double> expectedComDot = {-0.05018973433722229, 1.4166208451420528, 1.4301750486035787};
    std::vector<double> expectedComDdot = {-0.7606169667295027, 11.508107073695976, 16.58853835505851};

    for (unsigned int i=0; i<3; ++i) {
        EXPECT_NEAR(static_cast<double>(com(i, 0)), expectedCom[i], requiredPrecision);
        EXPECT_NEAR(static_cast<double>(comDot(i, 0)), expectedComDot[i],
                    requiredPrecision);
        EXPECT_NEAR(static_cast<double>(comDdot(i, 0)), expectedComDdot[i],
                    requiredPrecision);
    }
}

TEST(Segment, copy)
{
    Model model(modelPathForGeneralTesting);
    rigidbody::SegmentCharacteristics characteristics(
        10, utils::Vector3d(0.5, 0.5, 0.5),
        utils::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
    std::vector<utils::Range> ranges(6);
    rigidbody::Segment MasterSegment(
        model, "MasterSegment", "NoParent", "zyx", "yzx", ranges, ranges, ranges,
        characteristics, RigidBodyDynamics::Math::SpatialTransform());
    rigidbody::Segment ShallowCopy(MasterSegment);
    rigidbody::Segment ShallowCopyEqual = MasterSegment;
    rigidbody::Segment DeepCopyNow(MasterSegment.DeepCopy());
    rigidbody::Segment DeepCopyLater;
    DeepCopyLater.DeepCopy(MasterSegment);

    EXPECT_STREQ(MasterSegment.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopyEqual.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
    ShallowCopy.setParent("MyLovelyParent");
    EXPECT_STREQ(MasterSegment.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopyEqual.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
}

TEST(Mesh, copy)
{
    rigidbody::Mesh MasterMesh;
    MasterMesh.setPath("./MyFile.bioMesh");
    rigidbody::Mesh ShallowCopy(MasterMesh);
    rigidbody::Mesh DeepCopyNow(MasterMesh.DeepCopy());
    rigidbody::Mesh DeepCopyLater;
    DeepCopyLater.DeepCopy(MasterMesh);

    EXPECT_STREQ(MasterMesh.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyNow.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyLater.path().relativePath().c_str(), "./MyFile.bioMesh");
    ShallowCopy.setPath("./MyNewFile.bioMesh");
    EXPECT_STREQ(MasterMesh.path().relativePath().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().relativePath().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(DeepCopyNow.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyLater.path().relativePath().c_str(), "./MyFile.bioMesh");
}

static std::vector<std::vector<double>> expectedMarkers = {
    std::vector<double>({1.0126678074548392, 0.46575286691125295, -0.082379586527044829}),
    std::vector<double>({-0.18232123669751762,  0.98685937986570527,  0.46575286691125295}),
    std::vector<double>({0.39552020666133958, -0.18232123669751762,   1.0126678074548392}),
    std::vector<double>({1.0258667774186612, 1.0702910100794407, 1.1960410878390473})
};

TEST(Mesh, scale)
{
    rigidbody::Mesh mesh;
    mesh.addPoint(utils::Vector3d(2, 3, 4));
    mesh.addPoint(utils::Vector3d(5, 6, 7));

    mesh.scale(utils::Vector3d(2, 3, 4));

    {
        SCALAR_TO_DOUBLE(val, mesh.point(0)[0]);
        EXPECT_FLOAT_EQ(val, 4);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(0)[1]);
        EXPECT_FLOAT_EQ(val, 9);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(0)[2]);
        EXPECT_FLOAT_EQ(val, 16);
    }

    {
        SCALAR_TO_DOUBLE(val, mesh.point(1)[0]);
        EXPECT_FLOAT_EQ(val, 10);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(1)[1]);
        EXPECT_FLOAT_EQ(val, 18);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(1)[2]);
        EXPECT_FLOAT_EQ(val, 28);
    }

    mesh.rotate(utils::RotoTrans(
                    utils::Vector3d(0.2, 0.3, 0.4),
                    utils::Vector3d(2, 3, 4),
                    "xyz"));
    {
        SCALAR_TO_DOUBLE(val, mesh.point(0)[0]);
        EXPECT_FLOAT_EQ(val, 6.899786);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(0)[1]);
        EXPECT_FLOAT_EQ(val, 9.6247339);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(0)[2]);
        EXPECT_FLOAT_EQ(val, 20.885052);
    }

    {
        SCALAR_TO_DOUBLE(val, mesh.point(1)[0]);
        EXPECT_FLOAT_EQ(val, 12.377337);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(1)[1]);
        EXPECT_FLOAT_EQ(val, 17.880116);
    }
    {
        SCALAR_TO_DOUBLE(val, mesh.point(1)[2]);
        EXPECT_FLOAT_EQ(val, 33.64613);
    }
}

TEST(Mesh, color){
    rigidbody::Mesh mesh;
    utils::Vector3d color(mesh.color());
    {
        SCALAR_TO_DOUBLE(val, color[0]);
        EXPECT_FLOAT_EQ(val, 0.89);
    }
    {
        SCALAR_TO_DOUBLE(val, color[1]);
        EXPECT_FLOAT_EQ(val, 0.855);
    }
    {
        SCALAR_TO_DOUBLE(val, color[2]);
        EXPECT_FLOAT_EQ(val, 0.788);
    }

    mesh.setColor(utils::Vector3d(0.1, 0.2, 0.3));
    color = mesh.color();
    {
        SCALAR_TO_DOUBLE(val, color[0]);
        EXPECT_FLOAT_EQ(val, 0.1);
    }
    {
        SCALAR_TO_DOUBLE(val, color[1]);
        EXPECT_FLOAT_EQ(val, 0.2);
    }
    {
        SCALAR_TO_DOUBLE(val, color[2]);
        EXPECT_FLOAT_EQ(val, 0.3);
    }

}

TEST(Markers, allPositions)
{
    Model model(modelPathMeshEqualsMarker);
    EXPECT_EQ(model.nbQ(), 6);
    EXPECT_EQ(model.nbMarkers(), 4);

    rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Q[i] = QtestEqualsMarker[i];
    }

    // All markers at once
    std::vector<rigidbody::NodeSegment> markers(model.markers(Q, true,
            true));
    for (unsigned int i=0; i<model.nbMarkers(); ++i) {
        for (unsigned int j=0; j<3; ++j) {
            SCALAR_TO_DOUBLE(mark, markers[i][j]);
            EXPECT_NEAR(mark, expectedMarkers[i][j], requiredPrecision);
        }
    }
}

TEST(Markers, individualPositions)
{
    Model model(modelPathMeshEqualsMarker);
    EXPECT_EQ(model.nbQ(), 6);
    EXPECT_EQ(model.nbMarkers(), 4);

    rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Q[i] = QtestEqualsMarker[i];
    }

    // One marker at a time, only update Q once
    for (unsigned int i=0; i<model.nbMarkers(); ++i) {
        rigidbody::NodeSegment marker;
        if (i==0) {
            marker = model.marker(Q, i, true, true);
        } else {
            marker = model.marker(Q, i, true, false);
        }
        for (unsigned int j=0; j<3; ++j) {
            SCALAR_TO_DOUBLE(mark, marker[j]);
            EXPECT_NEAR(mark, expectedMarkers[i][j], requiredPrecision);
        }
    }

    // Change Q
    FILL_VECTOR(Q, std::vector<double>({0.3, 0.3, 0.3, 0.1, 0.1, 0.1}));
    std::vector<std::vector<double>> expectedMarkers2 = {
        std::vector<double>({1.290033288920621, 0.40925158443563553, 0.21112830525233722}),
        std::vector<double>({0.20066533460246938,  1.2890382781008347, 0.40925158443563558}),
        std::vector<double>({0.39983341664682814, 0.20066533460246938,  1.290033288920621}),
        std::vector<double>({1.2905320401699185, 1.2989551971389397, 1.310413178608594})
    };
    // One marker at a time, only update Q once
    for (unsigned int i=0; i<model.nbMarkers(); ++i) {
        rigidbody::NodeSegment marker;
        if (i==0) {
            marker = model.marker(Q, i, true, true);
        } else {
            marker = model.marker(Q, i, true, false);
        }
        for (unsigned int j=0; j<3; ++j) {
            SCALAR_TO_DOUBLE(mark, marker[j]);
            EXPECT_NEAR(mark, expectedMarkers2[i][j], requiredPrecision);
        }
    }
}

TEST(Mesh, position)
{
    Model model(modelPathMeshEqualsMarker);
    rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int q=0; q<model.nbQ(); ++q) {
        Q.setZero();
        Q[q] = 1;
        std::vector<std::vector<utils::Vector3d>> mesh(model.meshPoints(Q));
        std::vector<rigidbody::NodeSegment> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx) {
            for (unsigned int xyz =0; xyz<3; ++xyz) {
                SCALAR_TO_DOUBLE(meshDouble, mesh[0][idx][xyz]);
                SCALAR_TO_DOUBLE(markerDouble, markers[idx][xyz]);
                EXPECT_NEAR(meshDouble, markerDouble, requiredPrecision);
            }
        }
    }
    {
        Q.setOnes();
        std::vector<std::vector<utils::Vector3d>> mesh(model.meshPoints(Q));
        std::vector<rigidbody::NodeSegment> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx) {
            for (unsigned int xyz =0; xyz<3; ++xyz) {
                SCALAR_TO_DOUBLE(meshDouble, mesh[0][idx][xyz]);
                SCALAR_TO_DOUBLE(markerDouble, markers[idx][xyz]);
                EXPECT_NEAR(meshDouble, markerDouble, requiredPrecision);
            }
        }
    }
}

TEST(Dynamics, Forward)
{
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {
        20.554883896960259, -22.317642013324736, -77.406439058256126, 17.382961188212313,
        -63.426361095191858, 93.816468824985876, 106.46105024484631, 95.116641811710167,
        -268.1961283528546, 2680.3632159799949, -183.4582596257801, 755.89411812405604,
        163.60239754283589
    };

    CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);

    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }
}

TEST(Dynamics, ForwardDynamicsFreeFloatingBase)
{

    {
        Model model(modelPathForGeneralTesting);
        DECLARE_GENERALIZED_COORDINATES(Q, model);
        DECLARE_GENERALIZED_VELOCITY(QDot, model);
        DECLARE_GENERALIZED_OF_TYPE(Acceleration, QJointsDDot, model.nbQddot() - model.nbRoot());
     
        // Values from a Python script comparing the reference Python way to biorbd's way.
        // They were generated randomly.
        std::vector<double> valQ = {
            0.6772113773514055, 0.594725911263424, -0.37125218287187844, 0.8008719220322567,
            0.34387602931586936, -0.48585572458565385, -0.8771307019475285, 0.8971162532685075,
            0.5233833414396123, 0.4387214855367281, -0.261476841217011, -0.32085518311637085,
            0.6535149322727045
        };
        
        std::vector<double> valQDot = {
            -2.3439330676895542, 2.7424241596865895, -3.4950304777399976, -5.369662406205471,
            -1.8658120198345363, -5.02599152359938, -6.999413730817041, -0.7060849694543303,
            1.8995138906821407, -6.277466940454428, 9.054298192029442, -9.068984871797213,
            -4.4601279545571515
        };
        
        std::vector<double> valQJointsDDot = {
            -32.264957457954566, 8.302307945630583, -85.66701600446129, -33.16644219089828,
            3.3611301491870638, 35.10823095080686, -65.88196441641745, -53.43507412927335,
            81.06299775985526, 49.014951112137965
        };

        FILL_VECTOR(Q, valQ);
        FILL_VECTOR(QDot, valQDot);
        FILL_VECTOR(QJointsDDot, valQJointsDDot);

        std::vector<double> QRootDDot_expected = {
            0.4056651149671642, -13.250782774367915, 7.655292172975847
        };

        CALL_BIORBD_FUNCTION_3ARGS(QRootDDot, model, ForwardDynamicsFreeFloatingBase, Q, QDot, QJointsDDot);

        for (unsigned int i = 0; i<model.nbRoot(); ++i) {
            EXPECT_NEAR(static_cast<double>(QRootDDot(i, 0)), QRootDDot_expected[i],
                        requiredPrecision);
        }
    }
    
    {
        Model model(modelPathForGeneralTesting);
        DECLARE_GENERALIZED_COORDINATES(Q, model);
        DECLARE_GENERALIZED_VELOCITY(QDot, model);
        DECLARE_GENERALIZED_ACCELERATION(QJointsDDot, model);  // simulate possible mistake
        
        // Set to random values
        std::vector<double> valQ(model.nbQ());
        for (size_t i=0; i<valQ.size(); ++i) {
            valQ[i] = static_cast<double>(i) * 1.1;
        }
        std::vector<double> valQDot(model.nbQdot());
        for (size_t i=0; i<valQDot.size(); ++i) {
            valQDot[i] = static_cast<double>(i) * 1.1;
        }
        std::vector<double> valQDDot(model.nbQddot());
        for (size_t i=0; i<valQDDot.size(); ++i) {
            valQDDot[i] = static_cast<double>(i) * 1.1;
        }
        FILL_VECTOR(Q, valQ);
        FILL_VECTOR(QDot, valQDot);
        FILL_VECTOR(QJointsDDot, valQDDot);
        
        EXPECT_THROW(
            CALL_BIORBD_FUNCTION_3ARGS(QRootDDot, model, ForwardDynamicsFreeFloatingBase, Q, QDot, QJointsDDot),
            std::runtime_error);
    }
    
    {
        Model model(modelNoRootDoF);  // model without DoF on root
        DECLARE_GENERALIZED_COORDINATES(Q, model);
        DECLARE_GENERALIZED_VELOCITY(QDot, model);
        DECLARE_GENERALIZED_OF_TYPE(Acceleration, QJointsDDot, model.nbQddot() - model.nbRoot());
        
        // Set to random values
        std::vector<double> valQ(model.nbQ());
        for (size_t i=0; i<valQ.size(); ++i) {
            valQ[i] = static_cast<double>(i) * 1.1;
        }
        std::vector<double> valQDot(model.nbQdot());
        for (size_t i=0; i<valQDot.size(); ++i) {
            valQDot[i] = static_cast<double>(i) * 1.1;
        }
        std::vector<double> valQDDot(model.nbQddot());
        for (size_t i=0; i<valQDDot.size(); ++i) {
            valQDDot[i] = static_cast<double>(i) * 1.1;
        }
        FILL_VECTOR(Q, valQ);
        FILL_VECTOR(QDot, valQDot);
        FILL_VECTOR(QJointsDDot, valQDDot);
        
        EXPECT_THROW(
            CALL_BIORBD_FUNCTION_3ARGS(QRootDDot, model, ForwardDynamicsFreeFloatingBase, Q, QDot, QJointsDDot),
            std::runtime_error);
    }

}

#ifdef MODULE_ACTUATORS
TEST(Dynamics, ForwardChangingMass)
{
    Model model(modelSimple);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    rigidbody::SegmentCharacteristics c(model.segment(0).characteristics());
    c.setMass(10);
    model.updateSegmentCharacteristics(0, c);

    std::vector<double> QDDot_expected = {0.0, -9.7, 2.2};

    CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);

    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }


}
#endif


TEST(Dynamics, ForwardDynAndExternalForces)
{
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);
    std::vector<utils::SpatialVector> f_ext;
    for (size_t i=0; i<2; ++i) {
        double di = static_cast<double>(i);
        f_ext.push_back(utils::SpatialVector(
                            (di+1)*11.1, (di+1)*22.2, (di+1)*33.3, (di+1)*44.4, (di+1)*55.5, (di+1)*66.6));
    }

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {
        8.8871711208009998, -13.647827029817943, -33.606145294752132, 16.922669487341341,
        -21.882821189868423, 41.15364990805439, 68.892537246574463, -324.59756885799197,
        -447.99217990207387, 18884.241415786601, -331.24622725851572, 1364.7620674666462,
        3948.4748602722384
    };

    CALL_BIORBD_FUNCTION_3ARGS1PARAM(QDDot, model, ForwardDynamics, Q, QDot, Tau,
                                     &f_ext);

    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }
}


TEST(QDot, ComputeConstraintImpulsesDirect)
{
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);

    CALL_BIORBD_FUNCTION_2ARGS(QDotPost, model, ComputeConstraintImpulsesDirect, Q,
                               QDot);

    std::vector<double> QDotPost_expected = {
        0.92034698076739008, 0.4542331948818259, -1.1747551666658667, 3.3396871279100031,
        1.1143307751232683, 9.5534681791265204, 9.5313390358865036, 2.5590424787426884,
        -3.0502066043856577, 1.6659192923088271, 1.3562999563073794, -3.4457346325708458,
        3.2641898429292815
    };
    for (unsigned int i = 0; i<model.nbQdot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDotPost(i, 0)), QDotPost_expected[i],
                    requiredPrecision);
    }
}


TEST(Dynamics, ForwardLoopConstraint)
{
    Model model(modelPathForLoopConstraintTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {
        4357.563983223662,  -1980.272417081602, -4132.170113875329, 34854.96630091612,
        -5939.1875609623385, 20005.793234188295, -33019.84433234081, 5044.593964065896,
        76960.9024224599, 13949749.797541305, 29056.19402773685, 13957133.384121455
    };

    CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamicsConstraintsDirect, Q,
                               QDot, Tau);
    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i], 1e-2);
    }
}

TEST(Dynamics, ForwardAccelerationConstraint)
{
    Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_ACCELERATION(QDDot_constrained, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = 1.0;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {1.9402069774422919,  -9.1992692111538243,  2.9930159570454702,
                                          5.2738378853554133, 8.9387539396273699, 6.0938738229550751, 9.9560407885164217,
                                          38.6297746304162, -52.159023390563554, 36.702385054876714, 38.629774630416208, -52.159023390563561,
                                          36.70238505487675
                                         };
    std::vector<double> forces_expected = {-16.344680827308579, -30.485214214095951, 112.8234134576031, -16.344680827308611,
                                           -30.485214214095965, 112.82341345760311
                                          };

    rigidbody::Contacts cs(model.getConstraints());
    CALL_BIORBD_FUNCTION_3ARGS1PARAM(QDDot, model, ForwardDynamicsConstraintsDirect, Q, QDot, Tau, cs);
    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }
    EXPECT_EQ(cs.nbContacts(), forces_expected.size());

    CALL_BIORBD_FUNCTION_3ARGS(forces, model,
                               ContactForcesFromForwardDynamicsConstraintsDirect, Q, QDot, Tau);
    for (unsigned int i=0; i<forces_expected.size(); ++i) {
        EXPECT_NEAR(static_cast<double>(forces(i, 0)), forces_expected[i],
                    requiredPrecision);
    }
}

TEST(QuaternionInModel, sizes)
{
    Model m("models/simple_quat.bioMod");
    EXPECT_EQ(m.nbQ(), 4);
    EXPECT_EQ(m.nbQdot(), 3);
    EXPECT_EQ(m.nbQddot(), 3);
    EXPECT_EQ(m.nbGeneralizedTorque(), 3);
}

TEST(Kinematics, computeQdot)
{
    Model m("models/simple_quat.bioMod");
    DECLARE_GENERALIZED_VELOCITY(QDot, m);
    FILL_VECTOR(QDot, std::vector<double>({1, 2, 3}));
    {
        DECLARE_GENERALIZED_COORDINATES(Q_quat, m);
        FILL_VECTOR(Q_quat, std::vector<double>({0, 0, 0, 1}));
        std::vector<double> QDot_quat_expected = {0.5, 1, 1.5, 0};

        CALL_BIORBD_FUNCTION_2ARGS(QDot_quat, m, computeQdot, Q_quat, QDot);
        for (unsigned int i=0; i<m.nbQ(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDot_quat(i, 0)), QDot_quat_expected[i],
                        requiredPrecision);
        }
    }
    {
        double w(0.07035975447302918);
        double x(0.7035975447302919);
        double y(0.7035975447302919);
        double z(0.07035975447302918);
        DECLARE_GENERALIZED_COORDINATES(Q_quat, m);
        FILL_VECTOR(Q_quat, std::vector<double>({x, y, z, w}));
        std::vector<double> QDot_quat_expected = {1.0202164398589233, -0.9498566853858941,
                                                  0.45733840407468973,-1.1609359488049815
                                                 };
        CALL_BIORBD_FUNCTION_2ARGS(QDot_quat, m, computeQdot, Q_quat, QDot);

        for (unsigned int i=0; i<m.nbQ(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDot_quat(i, 0)), QDot_quat_expected[i],
                        requiredPrecision);
        }
    }
}

#ifdef MODULE_KALMAN
#ifndef SKIP_LONG_TESTS
TEST(Kalman, markers)
{
    Model model(modelPathForGeneralTesting);
    rigidbody::KalmanReconsMarkers kalman(model);
#ifdef BIORBD_USE_CASADI_MATH
    // Because the evaluation functions are not sym, it takes a LOOOONG time
    unsigned int nQToTest(1);
#else
    unsigned int nQToTest(model.nbQ());
#endif

    // Compute reference
    rigidbody::GeneralizedCoordinates Qref(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.2;
    }
    std::vector<rigidbody::NodeSegment> targetMarkers(model.markers(Qref));

    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    rigidbody::GeneralizedAcceleration Qddot(model);
    kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

    // Compare results (since the initialization of the filter is done 50X, it is expected to have converged)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        EXPECT_NEAR(q, qref, 1e-6);
        EXPECT_NEAR(qdot, 0, 1e-6);
        EXPECT_NEAR(qddot, 0, 1e-6);
    }

    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.3;
    }
    targetMarkers = model.markers(Qref);
    kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

    // Compare results (Here the filter should not have the time to converge)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        EXPECT_GT(fabs(q - qref), 1e-4);
        EXPECT_GT(fabs(qdot), 5);
        EXPECT_GT(fabs(qddot), 100);
    }

    // Force the filter to converge
    for (unsigned int i=0; i<100; ++i) {
        kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);
    }

    // Now it should be more or less equal
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        EXPECT_NEAR(q, qref, 1e-6);
        EXPECT_NEAR(qdot, 0, 1e-6);
        EXPECT_NEAR(qddot, 0, 1e-6);
    }
}
#endif

#ifndef SKIP_LONG_TESTS
TEST(Kalman, imu)
{
    Model model(modelPathForPyomecaman_withIMUs);
    rigidbody::KalmanReconsIMU kalman(model);
#ifdef BIORBD_USE_CASADI_MATH
    // Because the evaluation functions are not sym, it takes a LOOOONG time
    unsigned int nQToTest(3);

    // Test is known to fail for Kalman IMU (TODO: solve this)
    return;
#else
    unsigned int nQToTest(model.nbQ());
#endif

    // Compute reference
    rigidbody::GeneralizedCoordinates Qref(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.2;
    }
    std::vector<rigidbody::IMU> targetImus(model.IMU(Qref));

    rigidbody::GeneralizedCoordinates Q(model);
    rigidbody::GeneralizedVelocity Qdot(model);
    rigidbody::GeneralizedAcceleration Qddot(model);
    kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);

    // Compare results (since the initialization of the filter is done 50X, it is expected to have converged)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        if (i < 2) {
            // Translations are not reconstructed from IMU
            EXPECT_EQ(q, 0);
            EXPECT_EQ(qdot, 0);
            EXPECT_EQ(qddot, 0);
        } else {
            EXPECT_NEAR(q, qref, 1e-6);
            EXPECT_NEAR(qdot, 0, 1e-6);
            EXPECT_NEAR(qddot, 0, 1e-6);
        }
    }

    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.3;
    }
    targetImus = model.IMU(Qref);
    kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);

    // Compare results (Here the filter should not have the time to converge)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        if (i<2) {
            EXPECT_EQ(q, 0);
            EXPECT_EQ(qdot, 0);
            EXPECT_EQ(qddot, 0);
        } else {
            EXPECT_GT(abs(q - qref), 1e-4);
            EXPECT_GT(abs(qdot), 1e-2);
            EXPECT_GT(abs(qddot), 1e-1);
        }
    }

    // Force the filter to converge
    for (unsigned int i=0; i<1000; ++i) {
        kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);
    }

    // Now it should be more or less equal
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        if (i < 2) {
            // Translations are not reconstructed from IMU
            EXPECT_EQ(q, 0);
            EXPECT_EQ(qdot, 0);
            EXPECT_EQ(qddot, 0);
        } else {
            EXPECT_NEAR(q, qref, 1e-6);
            EXPECT_NEAR(qdot, 0, 1e-6);
            EXPECT_NEAR(qddot, 0, 1e-6);
        }
    }
}
#endif


#endif
