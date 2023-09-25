#ifndef BIORBD_UTILS_EXTERNAL_FORCE_H
#define BIORBD_UTILS_EXTERNAL_FORCE_H

#include <memory>

#include "biorbdConfig.h"
#include "Utils/Scalar.h"
#include "Utils/SpatialVector.h"
#include "Utils/RotoTransNode.h"
#include "RigidBody/RotoTransNodes.h"


namespace BIORBD_NAMESPACE
{
    class Model;
    namespace rigidbody {
        class GeneralizedCoordinates;
        class GeneralizedVelocity;
        class NodeSegment;
        class RotoTransNodes;
    }
    namespace utils
    {
        class Vector3d;
        class String;
     

        ///
        /// \brief An External force set that can apply forces to the model while computing the dynamics
        ///
        class BIORBD_API ExternalForceSet
        {

        protected:
#ifndef SWIG
            /// 
            /// This is an internal structure holding the information for the local forces. RotoTransNodes
            /// could have suffised, but we need to have a structure that was holding the spatial vector as well.
            class LocalForcesInternal : rigidbody::RotoTransNodes {
            public:
                void addRT(const utils::SpatialVector& vector, utils::RotoTransNode node) {
                    this->RotoTransNodes::addRT(node);
                    m_vectors.push_back(vector);
                }

                std::vector<utils::RotoTransNode> RTs(
                    const rigidbody::GeneralizedCoordinates& Q,
                    bool updateKin = true) {
                    return this->RotoTransNodes::RTs(Q, updateKin);
                }


                size_t size() const {
                    return m_RTs->size();
                }

                void clear() {
                    rigidbody::RotoTransNodes::clear();
                    m_vectors.clear();
                }

                std::vector<utils::SpatialVector> m_vectors; ///< All the force vectors, the size of which should always match the size of m_RTs
            };
#endif


        public:
            ///
            /// \brief Construct ExternalForceSet
            /// \param model The model onto which the external forces will be applied on. This is used 
            /// in conjunction with segmentName (when using set) to determine at which index the set vector 
            /// is positioned in the internal Set.
            /// \param useLinearForces If this force set has external forces 
            /// \param useSoftContacts If this force set has soft contacts
            ///
            ExternalForceSet(
                Model& model,
                bool useLinearForces = true,
                bool useSoftContacts = true
            );

            ///
            /// \brief Delete the set and free the ressources
            /// 
            ~ExternalForceSet();

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. WARNING: This vector 
            /// is expected to be acting on segmentName, applied at origin and expressed in the global reference frame.
            /// method offered by ExternalForceSet below.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param v The SpatialVector to add to the set.
            ///
            void add(
                const utils::String& segmentName,
                const utils::SpatialVector& v
            );

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. WARNING: This vector 
            /// is expected to be acting on segmentName, applied at pointOfApplication and expressed in the global reference frame. 
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param v The SpatialVector to add to the set.
            /// \param pointOfApplication Where the v vector is currenlty applied. 
            ///
            void add(
                const utils::String& segmentName,
                const utils::SpatialVector& v, 
                const utils::Vector3d& pointOfApplication
            );

#ifdef BIORBD_USE_CASADI_MATH
            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. Please see the 
            /// main "add" function for more precision.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param v The vector to add
            ///
            void add(
                utils::String& segmentName,
                const casadi::MX& v);

            ///
            /// \brief Apply a new value to the specified spatial vector of the Set. Please see the 
            /// main "add" function for more precision.
            /// \param segmentName The name of the segment to apply the spatial vector on.  
            /// \param m The vector to add
            ///
            void add(
                utils::String& segmentName,
                const RBDLCasadiMath::MX_Xd_SubMatrix& m);
#endif

            ///
            /// \brief Add a pure force at a specific point on the kinematic chain.
            /// \param force The pure forces to apply on the body
            /// \param segmentName The name of the segment the force is applied on
            /// \param pointOfApplication The position in the reference frame of [segmentName]
            /// 
            void addLinearForce(
                const utils::Vector3d& force,
                const utils::String& segmentName,
                const utils::Vector3d& pointOfApplication
            );

            ///
            /// \brief Add a pure force at a specific point on the kinematic chain. 
            /// \param forces The pure forces to apply on the body
            /// \param position The position of the force in reference frame of the node. 
            /// Note that the [parentName] of must have been set
            /// 
            void addLinearForce(
                const utils::Vector3d& force,
                const rigidbody::NodeSegment& pointOfApplication
            );


            ///
            /// \brief Initailize and zero out all the spatial vectors of the Set. Note this should always be called before
            /// reusing an ExternalForceSet unless one is absolutely sure that all the spatial vectors of the Set
            /// is in an expected state (that is, all the vectors were either set or zeroed)
            void setZero();

#ifndef SWIG

            /// 
            /// \brief The forces in a rbdl compatible format. This won't work if useLinearForces useSoftContacts is set to true
            /// \param Q The generalized coordinates
            /// \param QDot The generalized velocity
            /// \param updateKin If the kinematics of the model should be computed
            /// 
            std::vector<RigidBodyDynamics::Math::SpatialVector> computeRbdlSpatialVectors();

            /// 
            /// \brief The forces in a rbdl compatible format. This won't work if useSoftContacts is set to true
            /// \param Q The generalized coordinates
            /// \param QDot The generalized velocity
            /// \param updateKin If the kinematics of the model should be computed
            /// 
            std::vector<RigidBodyDynamics::Math::SpatialVector> computeRbdlSpatialVectors(
                const rigidbody::GeneralizedCoordinates& Q,
                bool updateKin = true
            );

            /// 
            /// \brief The forces in a rbdl compatible format. 
            /// \param Q The generalized coordinates
            /// \param QDot The generalized velocity
            /// \param updateKin If the kinematics of the model should be computed
            /// 
            std::vector<RigidBodyDynamics::Math::SpatialVector> computeRbdlSpatialVectors(
                const rigidbody::GeneralizedCoordinates& Q,
                const rigidbody::GeneralizedVelocity& QDot,
                bool updateKin = true
            );

            /// 
            /// \brief The forces in a rbdl compatible format. This won't work if useLinearForces or useSoftContacts is set to true
            /// 
            std::vector<utils::SpatialVector> computeSpatialVectors();

            /// 
            /// \brief The forces in a rbdl compatible format. This won't work if useSoftContacts is set to true
            /// \param Q The generalized coordinates
            /// \param updateKin If the kinematics of the model should be computed
            /// 
            std::vector<utils::SpatialVector> computeSpatialVectors(
                const rigidbody::GeneralizedCoordinates& Q,
                bool updateKin = true
            );

            /// 
            /// \brief The forces in a rbdl compatible format. 
            /// \param Q The generalized coordinates
            /// \param QDot The generalized velocity
            /// \param updateKin If the kinematics of the model should be computed
            /// 
            std::vector<utils::SpatialVector> computeSpatialVectors(
                const rigidbody::GeneralizedCoordinates& Q,
                const rigidbody::GeneralizedVelocity& QDot,
                bool updateKin = true
            );
#endif // !SWIG

            ///
            /// \brief Return if there is at least one external force expressed in the local reference frame.
            /// This is important since having this forces to send Q when computing the force in global reference frame
            /// 
            bool hasExternalForceInLocalReferenceFrame() const;

        protected:
            /// 
            /// \brief Add the forces expressed in the local reference to the internal Set.
            /// \param Q The Generalized coordinates. 
            /// \param out The vector of SpatialVector to fill
            /// 
            /// Note: as this is an internal method, even though Q is passed, it is assumed update kinematics was already done.
            /// 
            void combineLocalReferenceFrameForces(
                const rigidbody::GeneralizedCoordinates& Q,
                std::vector<utils::SpatialVector>& out
            );

            /// 
            /// \brief Add the linear forces to the internal Set.
            /// \param Q The Generalized coordinates. 
            /// \param out The vector of SpatialVector to fill
            /// 
            /// Note: as this is an internal method, even though Q is passed, it is assumed update kinematics was already done.
            /// 
            void combineLinearForces(
                const rigidbody::GeneralizedCoordinates& Q,
                std::vector<utils::SpatialVector>& out
            ) const;

            ///
            /// \brief Add the soft contact forces to the internal Set.
            /// \param Q The Generalized coordinates. 
            /// \param QDot The Generalized velocity. 
            /// \param out The vector of SpatialVector to fill
            /// 
            /// Note: as this is an internal method, even though Q is passed, it is assumed update kinematics was already done.
            /// 
            void combineSoftContactForces(
                const rigidbody::GeneralizedCoordinates& Q,
                const rigidbody::GeneralizedVelocity& QDot,
                std::vector<utils::SpatialVector>& out
            ) const;

            ///
            /// \brief Get the rigid contacts in a list of spatial vector of dimension 6xNdof
            /// \param force The force amplitude applied at the point
            /// \param pointOfApplication The position where the force is applied in base coordinate
            /// \return The effect of the force when transported at origin
            ///
            utils::SpatialVector transportForceAtOrigin(
                const utils::Vector3d& force,
                const rigidbody::NodeSegment& pointOfApplication
            ) const;

            ///
            /// \brief Get the rigid contacts in a list of spatial vector of dimension 6xNdof
            /// \param v The spatial vector to transport
            /// \param pointOfApplication The position where the force is applied in base coordinate
            /// \return The effect of the force when transported at origin
            ///
            utils::SpatialVector transportAtOrigin(
                const utils::SpatialVector& v,
                const rigidbody::NodeSegment& pointOfApplication
            ) const;
            
        protected:
            Model& m_model; ///< A reference to the kinematic model
            bool m_useLinearForces; ///< If linear forces should be included
            bool m_useSoftContacts; ///< If soft contacts should be included

            std::vector<utils::SpatialVector>
                m_externalForces; ///< The vector that holds all the external forces

            LocalForcesInternal m_externalForcesInLocal; ///< The vector that holds all the external forces that are expressed in local reference frame (must call Q).

            std::vector<std::pair<utils::Vector3d, rigidbody::NodeSegment>>
                m_linearForces; ///< The linear forces. The first is the amplitude and the second is the application point (that include the name of the parentSegment it is applied on)
        };
    }
}

#endif // BIORBD_UTILS_EXTERNAL_FORCE_H
