#ifndef BIORBD_UTILS_NODE3D_H
#define BIORBD_UTILS_NODE3D_H

#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class Attitude;
class String;

class BIORBD_API Node3d : public Eigen::Vector3d, public biorbd::utils::Node
{
    public:
    Node3d();
    Node3d(
            double x,
            double y,
            double z);
    template<typename OtherDerived> Node3d(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Vector3d(other), biorbd::utils::Node () {}
    template<typename OtherDerived> Node3d(
            const Eigen::MatrixBase<OtherDerived>& other, // Position du noeud
            const biorbd::utils::String &name,  // Nom du noeud
            const biorbd::utils::String &parentName) :
        Eigen::Vector3d(other), biorbd::utils::Node (name, parentName) {}
    Node3d(
            double x,
            double y,
            double z, // Position du noeud
            const biorbd::utils::String &name,  // Nom du noeud
            const biorbd::utils::String &parentName);
    biorbd::utils::Node3d DeepCopy() const;

    // Get and Set
    void setPosition(const biorbd::utils::Node3d& n);
    void setPosition(Eigen::Vector3d& n);
    void setPosition(Eigen::Vector4d& n);
    const biorbd::utils::Node3d& position() const;
    void applyRT(const Attitude&);
    const biorbd::utils::Node3d operator-(const biorbd::utils::Node3d &) const; // overload d'opérateurs
    const biorbd::utils::Node3d operator*(double) const; // overload d'opérateurs
    const biorbd::utils::Node3d operator/(double) const; // overload d'opérateurs

};

}}

#endif // BIORBD_UTILS_NODE3D_H
