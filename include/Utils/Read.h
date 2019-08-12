#ifndef S2M_READ_H
#define S2M_READ_H

#include <vector>
#include <Eigen/Dense>
#include <rbdl/rbdl_math.h>
#include "biorbdConfig.h"

class s2mMusculoSkeletalModel;
class s2mPath;
class s2mString;
class s2mNode;
namespace biorbd { namespace utils {
class GenCoord;
}}
class s2mBoneMesh;
class BIORBD_API s2mRead
{
public:
    static s2mMusculoSkeletalModel readModelFile(const s2mPath &path); // Open a model file
    static void readModelFile(const s2mPath &path, s2mMusculoSkeletalModel*); // Open a model file
    static std::vector<std::vector<Eigen::Vector3d>> readMarkerDataFile(const s2mString &path); // Lire un fichier de marqueurs
    static std::vector<biorbd::utils::GenCoord> readQDataFile(const s2mString &path);
    static std::vector<Eigen::VectorXd> readActivationDataFile(const s2mString &path);
    static std::vector<Eigen::VectorXd> readTorqueDataFile(const s2mString &path);
    static std::vector<Eigen::VectorXd> readGrfDataFile(const s2mString &path);

    static void readViconForceFile(const s2mString &path, // Path to the file
                                   std::vector<std::vector<unsigned int>> &time, // Frame vector (time is frame/frequency)
                                   std::vector<unsigned int> &frequency ,// Acquisition frequency
                                   std::vector<std::vector<Eigen::Vector3d>> &force, // Linear forces (x,y,z)
                                   std::vector<std::vector<Eigen::Vector3d>> &moment, // Moments (x,y,z)
                                   std::vector<std::vector<Eigen::Vector3d>> &cop); // Center of pressure (x,y,z)
    static std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector>>  readViconForceFile(const s2mString &path);
    /** Read a marker file CSV formated */
    static std::vector<std::vector<s2mNode>>  readViconMarkerFile(const s2mString &path, int nNodes =-1); // Path to the file, nombre de noeuds (-1 => tous) [gardes tous les markers]
    static std::vector<std::vector<s2mNode>>  readViconMarkerFile(const s2mString &path, std::vector<s2mString> &markOrder, int nNodes =-1); // Path to the file, markers a conserver, nombres de noeuds (-1 => tous)

    static s2mBoneMesh readBoneMeshFileS2mBones(const s2mPath& path);
    static s2mBoneMesh readBoneMeshFilePly(const s2mPath& path);

    static void pwd(); // Print the working directory
    static bool is_readable( const s2mString & file );

};

#endif // S2M_READ_H
