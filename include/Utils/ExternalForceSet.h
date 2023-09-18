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
            ///
            ExternalForceSet();

            ///
            /// \brief Delete the set and free the ressources
            /// 
            ~ExternalForceSet();

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. WARNING: This vector 
            /// is expected to be applied at origin in the global reference frame. If one needs to 
            /// transport and/or rotate the spatial vector, they can use the corresponding transformation 
            /// method offered by ExternalForceSet below.
            /// \param model The model onto which the external forces will be applied on. This is used 
            /// in conjunction with segmentName to determine at which index the new vector v is 
            /// positioned in the internal Set.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param v The SpatialVector to add to the set.
            /// \param from Where the v vector is currenlty applied. 
            ///
            void set(
                const rigidbody::Joints& model,
                const std::string& segmentName,
                const utils::SpatialVector& v
            );

#ifdef BIORBD_USE_CASADI_MATH
            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. Please see the 
            /// main "add" function for more precision.
            /// \param model The model onto which the external forces will be applied on. This is used 
            /// in conjunction with segmentName to determine at which index the new vector v is 
            /// positioned in the internal set.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param v The vector to add
            ///
            void set(
                rigidbody::Joints& model,
                std::string& segmentName,
                const casadi::MX& v);

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. Please see the 
            /// main "add" function for more precision.
            /// \param model The model onto which the external forces will be applied on. This is used 
            /// in conjunction with segmentName to determine at which index the new vector v is 
            /// positioned in the internal set.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param m The vector to add
            ///
            void set(
                rigidbody::Joints& model,
                std::string& segmentName,
                const RBDLCasadiMath::MX_Xd_SubMatrix& m);
#endif

            ///
            /// \brief Zero out all the spatial vectors of the Set. Note this should always be called before
            /// reusing an ExternalForceSet unless one is absolutely sure that all the spatial vectors of the Set
            /// is in an expected state (that is, all the vectors were either set or zeroed)
            void reset();

            /// 
            /// \brief The forces in a rbdl compatible format. It automatically adds the soft contact in the model
            /// unless the ignore soft contact is set to true, and adds the contactForces if there is any
            /// \param Q The generalized coordinates
            /// \param Qdot The generalized velocities
            /// \param contactForces The forces applied to the rigid contacts
            /// \param updateKin If the kinematics of the model should be computed
            /// \param ignoreSoftContact If the forces from soft contact in the model should be ignored or not
            /// 
            std::vector<RigidBodyDynamics::Math::SpatialVector>* toRbdl(
                const rigidbody::GeneralizedCoordinates& Q,
                const rigidbody::GeneralizedVelocity& QDot,
                std::vector<utils::Vector>* contactsForces = nullptr,
                bool updateKin = true,
                bool ignoreSoftContact = false
            ) const;

        protected:
            std::shared_ptr<std::vector<SpatialVector>> 
                m_vectors; ///< The vector that holds all the external forces

            //// -- FORCE PLATE DISPATCHER -- //
            /////
            ///// \brief Dispatch the forces from the force plate in a vector
            ///// \param spatialVector The values over time of one spatial vector per force platform
            ///// \param frame The frame to dispatch
            ///// \return A spatial vector with the forces
            /////
            //std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
            //    std::vector<std::vector<utils::SpatialVector>>& spatialVector,
            //    unsigned int frame) const;

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
                std::vector<utils::Vector>* contactsForces,
                bool updateKin
            ) const;


        };
    }
}

#endif // BIORBD_UTILS_EXTERNAL_FORCE_H
