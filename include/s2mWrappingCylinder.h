#ifndef S2M_WRAPPING_CYLINDER_H
#define S2M_WRAPPING_CYLINDER_H

#include "biorbdConfig.h"
#include "s2mWrappingObject.h"
#include "Utils/Attitude.h"

class BIORBD_API s2mWrappingCylinder : public s2mWrappingObject
{
    public:
        s2mWrappingCylinder(const s2mAttitude & = s2mAttitude(), // Position du centre
                            const double & = 0, // Diametre vue du dessus
                            const double & = 0, // Longueur du cylindre
                            const int & =1, // sens du wrapping (+1 ou -1)
                            const s2mString& = "",  // Nom du cylindre
                            const s2mString& = ""); // Nom du parent sur lequel il s'attache
        virtual ~s2mWrappingCylinder();

        void wrapPoints(const s2mAttitude&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr); // Premier et dernier points musculaire
        void wrapPoints(s2mJoints&, const s2mGenCoord&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) ; // Premier et dernier points musculaire
        void wrapPoints(s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr); // Premier et dernier points musculaire (si déja compute)


        // Set et get
        virtual s2mAttitude RT(s2mJoints &m, const s2mGenCoord& Q, const bool &updateKin = true);
        double diameter() const;
        double rayon() const;
        void setDiameter(double val);
        double length() const;
        void setLength(double val);

    protected:
        class NodeMusclePair{
        public:
            NodeMusclePair(const s2mNodeMuscle &p1, const s2mNodeMuscle &p2) : m_p1(p1),m_p2(p2){}
            ~NodeMusclePair(){}
            s2mNodeMuscle m_p1;
            s2mNodeMuscle m_p2;
        };

        // Trouve les deux tangentes d'un point avec un cercle
        void findTangentToCircle(const s2mNodeMuscle&, s2mNodeMuscle&) const;
        // Selectionne parmi un set de noeuds lesquels sont a garder
        void selectTangents(const NodeMusclePair&, s2mNodeMuscle&) const;
        // Trouver la hauteur des deux points (false = pas de wrap)
        bool findVerticalNode(const NodeMusclePair&, NodeMusclePair&) const;
        // Savoir s'il y a un wrapping qui doit etre fait
        bool checkIfWraps(const NodeMusclePair &, NodeMusclePair &) const;
        // Calcul de la longueur musculaire sur le cylindre
        double computeLength(const NodeMusclePair &) const;

        double m_dia; // diametre du cylindre
        double m_length; // Longueur du cylindre
        int m_side; // sens autours duquel passe les muscles
        s2mAttitude m_RTtoParent; // Matrice de rototrans avec le parent

        s2mNodeMuscle m_p1Wrap; // Premier point de contact avec le wrap
        s2mNodeMuscle m_p2Wrap; // Deuxieme point de contact avec le wrap
        double m_lengthAroundWrap ; // Longeur entre p1 et p2

};

#endif // S2M_WRAPPING_CYLINDER_H

