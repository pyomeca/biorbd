#ifndef BIORBD_UTILS_READ_H
#define BIORBD_UTILS_READ_H

#include <vector>
#include <rbdl/rbdl_math.h>
#include "biorbdConfig.h"

namespace biorbd {
class Model;

namespace rigidbody {
class GeneralizedCoordinates;
class BoneMesh;
}

namespace utils {
class Path;
class String;
class Node3d;
class Vector;
}

///
/// \brief Class Reader
///
class BIORBD_API Reader
{
public:
    ///
    /// \brief Open a model file
    /// \param path The path of the file
    ///
    static biorbd::Model readModelFile(const biorbd::utils::Path &path); // Open a model file

    ///
    /// \brief Open a model file
    /// \param path The path of the file
    /// \param model The model to fill
    /// \return Returns the model to fill
    ///
    static void readModelFile(const biorbd::utils::Path &path, biorbd::Model *model); // Open a model file

    ///
    /// \brief Read a file containing markers
    /// \param path The path of the file
    /// \return Returns the markers
    ///
    static std::vector<std::vector<biorbd::utils::Node3d>> readMarkerDataFile(const utils::Path &path); 

    /// 
    /// \brief Read a kin file
    /// \param path The path of the file
    /// \return Returns the kinematics
    /// 
    static std::vector<biorbd::rigidbody::GeneralizedCoordinates> readQDataFile(const biorbd::utils::Path &path);

    ///
    /// \brief Read an activation data file
    /// \param path The path of the file
    /// \return Returns the activations
    ///
    static std::vector<biorbd::utils::Vector> readActivationDataFile(const biorbd::utils::Path &path);

    ///
    /// \brief Read a torque data file
    /// \param path The path of the file
    /// \return Returns the torque
    ///
    static std::vector<biorbd::utils::Vector> readTorqueDataFile(const biorbd::utils::Path &path);

    ///
    /// \brief Read a ground reaction force file
    /// \param path The path of the file
    /// \return Returns the ground reaction force
    /// 
    static std::vector<biorbd::utils::Vector> readGroundReactionForceDataFile(const biorbd::utils::Path &path);

    /// 
    /// \brief Read a Vicon force file
    /// \param path The path of the file
    /// \param frame The fame vector
    /// \param frequency The acquisition frequency
    /// \param force The linear forces (x,y,z)
    /// \param moment The moments (x,y,z)
    /// \param cop The center of pressure (x,y,z)
    /// 
    static void readViconForceFile(
            const biorbd::utils::Path &path, // Path to the file
            std::vector<std::vector<unsigned int>> &frame, // Frame vector (time is frame/frequency)
            std::vector<unsigned int> &frequency ,// Acquisition frequency
            std::vector<std::vector<biorbd::utils::Node3d>> &force, // Linear forces (x,y,z)
            std::vector<std::vector<biorbd::utils::Node3d>> &moment, // Moments (x,y,z)
            std::vector<std::vector<biorbd::utils::Node3d>> &cop); // Center of pressure (x,y,z)

    ///
    /// \brief Read a Vicon force file
    /// \param path The path of the file
    /// \return Returns a Spatial Transform vector
    ///
    static std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>>  readViconForceFile(const biorbd::utils::String &path);

    ///
    /// \brief Read a Vicon marker file CSV formated
    /// \param path The path of the file
    /// \param nNodes The number of nodes (-1 => all) [keeps all of the markers]
    ///
    static std::vector<std::vector<biorbd::utils::Node3d>>  readViconMarkerFile(
            const biorbd::utils::Path &path,
            int nNodes =-1); // Path to the file, number of nodes (-1 => all) [keeps all of the markers]

    /// 
    /// \brief Read a Vicon marker file CSV formated
    /// \param path The path of the file
    /// \param markOrder The markers to keep
    /// \param nNodes The number of nodes (-1 => all) [keeps all of the markers]
    /// \return Returns the data
    ///
    static std::vector<std::vector<biorbd::utils::Node3d>>  readViconMarkerFile(
            const biorbd::utils::Path &path,
            std::vector<biorbd::utils::String> &markOrder,
            int nNodes =-1); // Path to the file, markers to keep, number of nodes (-1 => all)

    ///
    /// \brief Read a biorbd bone mesh file
    /// \param path The path of the file
    /// \return Returns the mesh
    ///
    static biorbd::rigidbody::BoneMesh readBoneMeshFileBiorbdBones(const biorbd::utils::Path& path);

    ///
    /// \brief Read a PLY bone mesh file
    /// \param path The path of the file
    /// \return Returns the mesh
    ///
    static biorbd::rigidbody::BoneMesh readBoneMeshFilePly(const biorbd::utils::Path& path);
#ifdef MODULE_VTP_FILES_READER
    static biorbd::rigidbody::BoneMesh readBoneMeshFileVtp(const biorbd::utils::Path& path);
#endif

};

}

#endif // BIORBD_UTILS_READ_H
