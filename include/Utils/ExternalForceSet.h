#ifndef BIORBD_UTILS_EXTERNAL_FORCE_H
#define BIORBD_UTILS_EXTERNAL_FORCE_H

#include "biorbdConfig.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
    class Model;
    namespace rigidbody {
        class GeneralizedCoordinates;
        class GeneralizedVelocity;
        class NodeSegment;
    }
    namespace utils
    {
        class SpatialVector;
        class Vector3d;
        class String;

        ///
        /// \brief An External force set that can apply forces to the model while computing the dynamics
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
                Model& model);

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
                const utils::String& segmentName,
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
                utils::String& segmentName,
                const casadi::MX& v);

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. Please see the 
            /// main "add" function for more precision.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param m The vector to add
            ///
            void set(
                utils::String& segmentName,
                const RBDLCasadiMath::MX_Xd_SubMatrix& m);
#endif

            ///
            /// \brief Add a pure force at a specific point on the kinematic chain. Note that
            /// contrary to [set], [addExternalPush] expect a call to [applyForces] to finalize
            /// the add. Otherwise the forces won't be included.
            /// \param forces The pure forces to apply on the body
            /// \param segmentName The name of the segment the force is applied on
            /// \param position The position in the reference frame of [segmentName]
            /// 
            void addExternalPush(
                const utils::Vector3d& forces,
                const utils::String& segmentName,
                const utils::Vector3d& position
            );

            ///
            /// \brief Add a pure force at a specific point on the kinematic chain. Note that
            /// contrary to [set], [addExternalPush] expect a call to [applyForces] to finalize
            /// the add. Otherwise the forces won't be included.
            /// \param forces The pure forces to apply on the body
            /// \param position The position of the force in reference frame of the node. 
            /// Note that the [parentName] of must have been set
            /// 
            void addExternalPush(
                const utils::Vector3d& forces,
                const rigidbody::NodeSegment& position
            );


            ///
            /// \brief Initailize and zero out all the spatial vectors of the Set. Note this should always be called before
            /// reusing an ExternalForceSet unless one is absolutely sure that all the spatial vectors of the Set
            /// is in an expected state (that is, all the vectors were either set or zeroed)
            void reset();

#ifndef SWIG
            /// 
            /// \brief The forces in a rbdl compatible format. 
            /// If one wants to add push forces, they should call [addExternalPush] and [applyForces] before
            /// calling [toRbld]. To include softContact, they should alse call [applyForces]. Otherwise both
            /// push forces and soft contact forces are effectively ignored. Alternatively, if one is only 
            /// interested in push forces, they can call [applyPushForces]
            /// IMPORTANT: The user should NOT destruct the pointer returned by this method.
            /// 
            std::vector<RigidBodyDynamics::Math::SpatialVector>* toRbdl() const;
#endif // !SWIG

            ///
            /// \brief Interface to apply all previously added force
            /// \param Q The generalized coordinates
            /// \param QDot The generalized velocity
            /// \param updateKin If the kinematics of the model should be computed
            /// \param includeSoftContact Set to false if soft contact should be ignored (that is automatically 
            /// done if the model does not include any soft contacts)
            /// \param includePushForces Set to false if push forces should be ignored (that is automatically
            /// done if no push forces were added
            ///
            void utils::ExternalForceSet::applyForces(
                const rigidbody::GeneralizedCoordinates& Q,
                const rigidbody::GeneralizedVelocity& QDot,
                bool updateKin = true,
                bool includeSoftContact = true,
                bool includePushForces = true
            );

            ///
            /// \brief Interface to apply all previously added force
            /// \param Q The generalized coordinates
            /// \param updateKin If the kinematics of the model should be computed
            ///
            void utils::ExternalForceSet::applyPushForces(
                const rigidbody::GeneralizedCoordinates& Q,
                bool updateKin = true
            );

        protected:
            /////
            ///// \brief Dispatch the forces from the force plate in a spatial vector
            ///// \param sv One spatial vector per force platform
            ///// \return A spatial vector with the forces
            /////
            //std::vector<RigidBodyDynamics::Math::SpatialVector>* dispatchedForce(
            //    std::vector<utils::SpatialVector>* sv) const;



            /// 
            /// \brief Add the contact forces from rigid and soft contacts to the internal Set.
            /// If the vector is reset, this method should be called again.
            /// \param rigidContactForces The forces to add to the rigid contact. If none is provided
            /// then only soft contact are added
            /// \param Q The Generalized coordinates
            /// 
            void utils::ExternalForceSet::combineExternalPushes(const rigidbody::GeneralizedCoordinates& Q);

            /////
            ///// \brief Get the rigid contacts in a list of spatial vector of dimension 6xNdof
            ///// \param f_contacts the forces applied at the contact points
            ///// \param Q The Generalized coordinates
            ///// \return The rigid contacts
            /////
            //std::vector<utils::SpatialVector>& rigidContactToSpatialVector(
            //    std::vector<utils::Vector> f_contacts,
            //    const rididbody::GeneralizedCoordinates& Q,
            //    bool updateKin);


            ///
            /// \brief Get the rigid contacts in a list of spatial vector of dimension 6xNdof
            /// \param forces The force amplitude applied at the point
            /// \param position The position where the force is applied in base coordinate
            /// \return The effect of the force when transported at origin
            ///
            utils::SpatialVector transportForceAtOrigin(
                const utils::Vector3d& forces,
                const rigidbody::NodeSegment& position
            );



            Model& 
                m_model; ///< A reference to the kinematic model

            std::shared_ptr<std::vector<SpatialVector>>
                m_vectors; ///< The vector that holds all the external forces

            std::vector<RigidBodyDynamics::Math::SpatialVector>*
                m_rbdlFormattedForces; ///< The external vector as it is expected by RBDL (returned by toRbdl)

            std::vector<std::pair<utils::Vector3d, rigidbody::NodeSegment>>
                m_externalPush; ///< The push from pure forces. The first is the amplitude and the second is the application point (that include the name of the parentSegment it is applied on)
        };
    }
}

#endif // BIORBD_UTILS_EXTERNAL_FORCE_H
