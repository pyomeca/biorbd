#ifndef BIORBD_MUSCLES_WRAPPING_CYLINDER_H
#define BIORBD_MUSCLES_WRAPPING_CYLINDER_H

#include "biorbdConfig.h"
#include "Muscles/WrappingObject.h"

namespace biorbd {
namespace muscles {

class BIORBD_API WrappingCylinder : public biorbd::muscles::WrappingObject
{
public:
    WrappingCylinder();
    WrappingCylinder(
            const biorbd::utils::RotoTrans& rt,
            double diameter,
            double length,
            bool isCylinderPositiveSign);
    WrappingCylinder(
            const biorbd::utils::RotoTrans& rt,
            double diameter,
            double length,
            bool isCylinderPositiveSign,
            const biorbd::utils::String& name,
            const biorbd::utils::String& parentName);
    biorbd::muscles::WrappingCylinder DeepCopy() const;
    void DeepCopy(const biorbd::muscles::WrappingCylinder& other);

    void wrapPoints(
            const biorbd::utils::RotoTrans& rt,
            const biorbd::utils::Node3d& p1_bone,
            const biorbd::utils::Node3d& p2_bone,
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* length = nullptr); // Premier et dernier points musculaire
    void wrapPoints(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::utils::Node3d& p1_bone,
            const biorbd::utils::Node3d& p2_bone,
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* length = nullptr) ; // Premier et dernier points musculaire
    void wrapPoints(
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* length = nullptr); // Premier et dernier points musculaire (si déja compute)

    // Set et get
    virtual const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool updateKin = true);
    double diameter() const;
    void setDiameter(double val);
    double rayon() const;
    double length() const;
    void setLength(double val);

protected:
    class NodeMusclePair{
    public:
        NodeMusclePair(
                const biorbd::utils::Node3d &p1,
                const biorbd::utils::Node3d &p2) :
            m_p1(std::make_shared<biorbd::utils::Node3d>(p1)),
            m_p2(std::make_shared<biorbd::utils::Node3d>(p2))
        {}
        std::shared_ptr<biorbd::utils::Node3d> m_p1;
        std::shared_ptr<biorbd::utils::Node3d> m_p2;
    };

    // Trouve les deux tangentes d'un point avec un cercle
    void findTangentToCircle(
            const biorbd::utils::Node3d&,
            biorbd::utils::Node3d&) const;
    // Selectionne parmi un set de noeuds lesquels sont a garder
    void selectTangents(
            const NodeMusclePair&, biorbd::utils::Node3d&) const;
    // Trouver la hauteur des deux points (false = pas de wrap)
    bool findVerticalNode(
            const NodeMusclePair&,
            NodeMusclePair&) const;
    // Savoir s'il y a un wrapping qui doit etre fait
    bool checkIfWraps(
            const NodeMusclePair &,
            NodeMusclePair &) const;
    // Calcul de la longueur musculaire sur le cylindre
    double computeLength(
            const NodeMusclePair &) const;

    std::shared_ptr<double> m_dia; // diametre du cylindre
    std::shared_ptr<double> m_length; // Longueur du cylindre
    std::shared_ptr<bool> m_isCylinderPositiveSign; // sens autours duquel passe les muscles
    std::shared_ptr<biorbd::utils::RotoTrans> m_RTtoParent; // Matrice de rototrans avec le parent

    std::shared_ptr<biorbd::utils::Node3d> m_p1Wrap; // Premier point de contact avec le wrap
    std::shared_ptr<biorbd::utils::Node3d> m_p2Wrap; // Deuxieme point de contact avec le wrap
    std::shared_ptr<double> m_lengthAroundWrap ; // Longeur entre p1 et p2

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_CYLINDER_H

