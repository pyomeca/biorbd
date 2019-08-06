#ifndef S2MNODEBONE_H
#define S2MNODEBONE_H
	#include "biorbdConfig.h"
    #include <Eigen/Dense>
    #include "s2mNode.h"

class s2mNode;
class BIORBD_API s2mNodeBone : public s2mNode
{ 
    public:
        s2mNodeBone(const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
                    const s2mString& = "", // Nom du noeud
                    const s2mString& = "", // Nom du parent
                    bool = true, // Si le marker est un marker technique
                    bool = true, // Si le marker est un marker anatomique
                    const s2mString& axesToRemove = s2mString(), // Axes à retirer
                    int = -1); // Numéro ID du parent
        virtual ~s2mNodeBone();

        // Get and Set
        virtual bool isTechnical() const;
        virtual bool isAnatomical() const;
        int parentId() const;
        virtual const s2mNodeBone &position() const;
        virtual s2mNodeBone position(bool removeAxes) const;

        bool isAxisRemoved(unsigned int) const;
        bool isAxisKept(unsigned int) const;
        void addAxesToRemove(unsigned int); // Ajouter un axes à retirer
        void addAxesToRemove(s2mString); // Ajouter un axes à retirer
        void addAxesToRemove(std::vector<unsigned int>); // Ajouter un axes à retirer
        void addAxesToRemove(std::vector<s2mString>); // Ajouter un axes à retirer
        s2mString axesToRemove();
        int nAxesToRemove() const; // Nombre d'axes à retirer
    protected:
        std::vector<bool> m_axesRemoved; // Matrice de projection
        int m_nbAxesToRemove; // Retirer un ou plusieurs axes (1 axe : projeter sur un plan, 2 axes : projeter sur le 3ième axes, 3 axes : retourne la position du repère parent
        bool m_technical; // If a marker is a technical marker
        bool m_anatomical; // It marker is a anatomical marker
        int m_id;
    private:

};

#endif // S2MNODEBONE_H
