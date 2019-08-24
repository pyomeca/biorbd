#ifndef BIORBD_UTILS_NODE3D_H
#define BIORBD_UTILS_NODE3D_H

#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class RotoTrans;
class String;

class BIORBD_API Node3d : public Eigen::Vector3d, public biorbd::utils::Node
{
    public:
    Node3d();
    Node3d(
            double x,
            double y,
            double z);
    Node3d(const Eigen::Vector4d& v);
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
    biorbd::utils::Node3d applyRT(const RotoTrans&) const;
    void applyRT(const RotoTrans&);

    biorbd::utils::Node3d& operator=(const Eigen::Vector4d& other);
    template<typename OtherDerived>
        biorbd::utils::Node3d& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::Vector3d::operator=(other);
            return *this;
        }

};

}}

#endif // BIORBD_UTILS_NODE3D_H
