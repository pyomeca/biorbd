#ifndef BIORBD_RIGIDBODY_NODE_BONE_H
#define BIORBD_RIGIDBODY_NODE_BONE_H

#include <vector>
#include "biorbdConfig.h"
#include "Utils/Node3d.h"

#include "Utils/String.h"
namespace biorbd {
namespace rigidbody {

class BIORBD_API NodeBone : public biorbd::utils::Node3d
{ 
public:
    NodeBone(
            double x = 0,
            double y = 0,
            double z = 0,
            const biorbd::utils::String& name= "", // Nom du noeud
            const biorbd::utils::String& parentName= "", // Nom du parent
            bool isTechnical = true, // Si le marker est un marker technique
            bool isAnatomical = true, // Si le marker est un marker anatomique
            const biorbd::utils::String& axesToRemove = biorbd::utils::String(), // Axes à retirer
            int parentID = -1); // Numéro ID du parent
    NodeBone(
            const biorbd::utils::Node3d& node, // Position
            const biorbd::utils::String& name= "", // Nom du noeud
            const biorbd::utils::String& parentName= "", // Nom du parent
            bool isTechnical = true, // Si le marker est un marker technique
            bool isAnatomical = true, // Si le marker est un marker anatomique
            const biorbd::utils::String& axesToRemove = biorbd::utils::String(), // Axes à retirer
            int parentID = -1); // Numéro ID du parent

    // Get and Set
    bool isTechnical() const;
    bool isAnatomical() const;
    int parentId() const;

    NodeBone removeAxes() const;
    bool isAxisRemoved(unsigned int) const;
    bool isAxisKept(unsigned int) const;
    void addAxesToRemove(unsigned int); // Ajouter un axes à retirer
    void addAxesToRemove(const biorbd::utils::String& axis); // Ajouter un axes à retirer
    void addAxesToRemove(const std::vector<unsigned int>& axis); // Ajouter un axes à retirer
    void addAxesToRemove(const std::vector<biorbd::utils::String>& axis); // Ajouter un axes à retirer
    biorbd::utils::String axesToRemove();
    int nAxesToRemove() const; // Nombre d'axes à retirer

    template<typename OtherDerived>
        biorbd::rigidbody::NodeBone& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::Node3d::operator=(other);
            return *this;
        }

protected:
    std::vector<bool> m_axesRemoved; // Matrice de projection
    int m_nbAxesToRemove; // Retirer un ou plusieurs axes (1 axe : projeter sur un plan, 2 axes : projeter sur le 3ième axes, 3 axes : retourne la position du repère parent
    bool m_technical; // If a marker is a technical marker
    bool m_anatomical; // It marker is a anatomical marker
    int m_id;

};

}}

#endif // BIORBD_RIGIDBODY_NODE_BONE_H
