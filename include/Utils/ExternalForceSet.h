#ifndef BIORBD_UTILS_EXTERNAL_FORCE_H
#define BIORBD_UTILS_EXTERNAL_FORCE_H

#include "biorbdConfig.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
    namespace rigidbody {
        class Joints;
        class GeneralizedCoordinates;
        class GeneralizedVelocity;
    }
    namespace utils
    {
        class SpatialVector;
        class Vector;

        ///
        /// \brief Wrapper of the Eigen::Matrix<double, 6, 1> or Casadi::MX(6, 1)
        ///
        class BIORBD_API ExternalForceSet
        {
        public:
            ///
            /// \brief Construct ExternalForceSet
            /// \param model The model onto which the external forces will be applied on. This is used 
            /// in conjunction with segmentName (when using set) to determine at which index the set vector 
            /// is positioned in the internal Set.
            ///
            ExternalForceSet(
                const rigidbody::Joints& model);

            ///
            /// \brief Delete the set and free the ressources
            /// 
            ~ExternalForceSet();

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. WARNING: This vector 
            /// is expected to be applied at origin in the global reference frame. If one needs to 
            /// transport and/or rotate the spatial vector, they can use the corresponding transformation 
            /// method offered by ExternalForceSet below.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param v The SpatialVector to add to the set.
            /// \param from Where the v vector is currenlty applied. 
            ///
            void set(
                const std::string& segmentName,
                const utils::SpatialVector& v
            );

#ifdef BIORBD_USE_CASADI_MATH
            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. Please see the 
            /// main "add" function for more precision.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param v The vector to add
            ///
            void set(
                std::string& segmentName,
                const casadi::MX& v);

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. Please see the 
            /// main "add" function for more precision.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param m The vector to add
            ///
            void set(
                std::string& segmentName,
                const RBDLCasadiMath::MX_Xd_SubMatrix& m);
#endif

            ///
            /// \brief Initailize and zero out all the spatial vectors of the Set. Note this should always be called before
            /// reusing an ExternalForceSet unless one is absolutely sure that all the spatial vectors of the Set
            /// is in an expected state (that is, all the vectors were either set or zeroed)
            void reset();

#ifndef SWIG
            /// 
            /// \brief The forces in a rbdl compatible format. One can add a force vector [externallyComputedForces] 
            /// of dimension XXX that will be added to the internal set. 
            /// IMPORTANT: The user should NOT destruct the pointer returned by this method.
            /// \param externallyComputedForces The forces applied to the rigid contacts
            /// 
            std::vector<RigidBodyDynamics::Math::SpatialVector>* toRbdl(
                const std::vector<utils::Vector>& externallyComputedForces = std::vector<utils::Vector>()
            ) const;

            /// 
            /// \brief The forces in a rbdl compatible format. It adds the forces frome the soft contact in the model
            /// One can add a force vector [externallyComputedForces] of dimension XXX that will be added 
            /// to the internal set.
            /// IMPORTANT: The user should NOT destruct the pointer returned by this method.
            /// \param Q The generalized coordinates
            /// \param Qdot The generalized velocities
            /// \param updateKin If the kinematics of the model should be computed
            /// \param externallyComputedForces The forces applied to the rigid contacts
            /// 
            std::vector<RigidBodyDynamics::Math::SpatialVector>* toRbdlWithSoftContact(
                const rigidbody::GeneralizedCoordinates& Q,
                const rigidbody::GeneralizedVelocity& QDot,
                bool updateKin = true,
                const std::vector<utils::Vector>& externallyComputedForces = std::vector<utils::Vector>()
            ) const;
#endif // !SWIG

        protected:
            /////
            ///// \brief Dispatch the forces from the force plate in a spatial vector
            ///// \param sv One spatial vector per force platform
            ///// \return A spatial vector with the forces
            /////
            //std::vector<RigidBodyDynamics::Math::SpatialVector>* dispatchedForce(
            //    std::vector<utils::SpatialVector>* sv) const;


            ///
            /// \brief Interface to combine to vectors of RigidBodyDynamics::Math::SpatialVector
            /// \param Q The generalized coordinates
            /// \param Qdot The generalized velocities
            /// \param contactForces The forces applied to the rigid contacts
            /// \param updateKin If the kinematics of the model should be computed
            ///
            utils::ExternalForceSet& combineExtForceAndSoftContact(
                const rigidbody::GeneralizedCoordinates& Q,
                const rigidbody::GeneralizedVelocity& QDot,
                bool updateKin,
                const std::vector<utils::Vector>& externallyComputedForces = std::vector<utils::Vector>()
            ) const;


            const rigidbody::Joints& 
                m_model; ///< A reference to the kinematic model

            std::shared_ptr<std::vector<SpatialVector>>
                m_vectors; ///< The vector that holds all the external forces

            std::vector<RigidBodyDynamics::Math::SpatialVector>* 
                m_rbdlFormattedForces; ///< The external vector as it is expected by RBDL (returned by toRbdl)

        };
    }
}

#endif // BIORBD_UTILS_EXTERNAL_FORCE_H
