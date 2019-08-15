#ifndef BIORBD_MUSCLES_WRAPPING_CYLINDER_H
#define BIORBD_MUSCLES_WRAPPING_CYLINDER_H

#include "biorbdConfig.h"
#include "Utils/Attitude.h"
#include "Muscles/WrappingObject.h"

namespace biorbd {
namespace muscles {

class BIORBD_API WrappingCylinder : public biorbd::muscles::WrappingObject
{
public:
    WrappingCylinder(
            const biorbd::utils::Attitude & = biorbd::utils::Attitude(), // Position du centre
            const double & = 0, // Diametre vue du dessus
            const double & = 0, // Longueur du cylindre
            const int & =1, // sens du wrapping (+1 ou -1)
            const biorbd::utils::String& = "",  // Nom du cylindre
            const biorbd::utils::String& = ""); // Nom du parent sur lequel il s'attache
    virtual ~WrappingCylinder();

    void wrapPoints(
            const biorbd::utils::Attitude&,
            const biorbd::muscles::MuscleNode&,
            const biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            double* = nullptr); // Premier et dernier points musculaire
    void wrapPoints(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const biorbd::muscles::MuscleNode&,
            const biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            double* = nullptr) ; // Premier et dernier points musculaire
    void wrapPoints(
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            double* = nullptr); // Premier et dernier points musculaire (si déja compute)


    // Set et get
    virtual biorbd::utils::Attitude RT(
            biorbd::rigidbody::Joints &m,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const bool &updateKin = true);
    double diameter() const;
    double rayon() const;
    void setDiameter(double val);
    double length() const;
    void setLength(double val);

protected:
    class NodeMusclePair{
    public:
        NodeMusclePair(const biorbd::muscles::MuscleNode &p1, const biorbd::muscles::MuscleNode &p2) : m_p1(p1),m_p2(p2){}
        ~NodeMusclePair(){}
        biorbd::muscles::MuscleNode m_p1;
        biorbd::muscles::MuscleNode m_p2;
    };

    // Trouve les deux tangentes d'un point avec un cercle
    void findTangentToCircle(const biorbd::muscles::MuscleNode&, biorbd::muscles::MuscleNode&) const;
    // Selectionne parmi un set de noeuds lesquels sont a garder
    void selectTangents(const NodeMusclePair&, biorbd::muscles::MuscleNode&) const;
    // Trouver la hauteur des deux points (false = pas de wrap)
    bool findVerticalNode(const NodeMusclePair&, NodeMusclePair&) const;
    // Savoir s'il y a un wrapping qui doit etre fait
    bool checkIfWraps(const NodeMusclePair &, NodeMusclePair &) const;
    // Calcul de la longueur musculaire sur le cylindre
    double computeLength(const NodeMusclePair &) const;

    double m_dia; // diametre du cylindre
    double m_length; // Longueur du cylindre
    int m_side; // sens autours duquel passe les muscles
    biorbd::utils::Attitude m_RTtoParent; // Matrice de rototrans avec le parent

    biorbd::muscles::MuscleNode m_p1Wrap; // Premier point de contact avec le wrap
    biorbd::muscles::MuscleNode m_p2Wrap; // Deuxieme point de contact avec le wrap
    double m_lengthAroundWrap ; // Longeur entre p1 et p2

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_CYLINDER_H

