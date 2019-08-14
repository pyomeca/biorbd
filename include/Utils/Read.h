#ifndef BIORBD_UTILS_READ_H
#define BIORBD_UTILS_READ_H

#include <vector>
#include <Eigen/Dense>
#include <rbdl/rbdl_math.h>
#include "biorbdConfig.h"

class s2mMusculoSkeletalModel;

namespace biorbd { namespace rigidbody {
class Mesh;
}}

namespace biorbd { namespace utils {
class Path;
class String;
class Node;
class GenCoord;

class BIORBD_API Read
{
public:
    static s2mMusculoSkeletalModel readModelFile(const biorbd::utils::Path &path); // Open a model file
    static void readModelFile(const biorbd::utils::Path &path, s2mMusculoSkeletalModel*); // Open a model file
    static std::vector<std::vector<Eigen::Vector3d>> readMarkerDataFile(const biorbd::utils::String &path); // Lire un fichier de marqueurs
    static std::vector<biorbd::utils::GenCoord> readQDataFile(const biorbd::utils::String &path);
    static std::vector<Eigen::VectorXd> readActivationDataFile(const biorbd::utils::String &path);
    static std::vector<Eigen::VectorXd> readTorqueDataFile(const biorbd::utils::String &path);
    static std::vector<Eigen::VectorXd> readGrfDataFile(const biorbd::utils::String &path);

    static void readViconForceFile(
            const biorbd::utils::String &path, // Path to the file
            std::vector<std::vector<unsigned int>> &time, // Frame vector (time is frame/frequency)
            std::vector<unsigned int> &frequency ,// Acquisition frequency
            std::vector<std::vector<Eigen::Vector3d>> &force, // Linear forces (x,y,z)
            std::vector<std::vector<Eigen::Vector3d>> &moment, // Moments (x,y,z)
            std::vector<std::vector<Eigen::Vector3d>> &cop); // Center of pressure (x,y,z)
    static std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>>  readViconForceFile(const biorbd::utils::String &path);
    // Read a marker file CSV formated
    static std::vector<std::vector<Node>>  readViconMarkerFile(
            const biorbd::utils::String &path,
            int nNodes =-1); // Path to the file, nombre de noeuds (-1 => tous) [gardes tous les markers]
    static std::vector<std::vector<Node>>  readViconMarkerFile(
            const biorbd::utils::String &path,
            std::vector<biorbd::utils::String> &markOrder,
            int nNodes =-1); // Path to the file, markers a conserver, nombres de noeuds (-1 => tous)

    static biorbd::rigidbody::Mesh readBoneMeshFileS2mBones(const biorbd::utils::Path& path);
    static biorbd::rigidbody::Mesh readBoneMeshFilePly(const biorbd::utils::Path& path);

    static void pwd(); // Print the working directory
    static bool is_readable( const biorbd::utils::String & file );

};

}}

#endif // BIORBD_UTILS_READ_H
