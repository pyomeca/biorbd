#ifndef S2M_IMU_H
#define S2M_IMU_H

#include "biorbdConfig.h"
#include "s2mNodeAttitude.h"

class BIORBD_API s2mIMU : public s2mNodeAttitude
{ 
    public:
        s2mIMU(const s2mAttitude& = s2mAttitude(), // Position
                    const s2mString& = "", // Nom du noeud
                    const s2mString& = "", // Nom du parent
                    const bool& = true, // Si le marker est un marker technique
                    const bool& = true, // Si le marker est un marker anatomique
                    const int& = -1); // Numéro ID du parent

        virtual ~s2mIMU();
        // Get and Set
        virtual bool isTechnical() const;
        virtual bool isAnatomical() const;
        int parentId() const;
    protected:
        bool m_technical; // If a marker is a technical marker
        bool m_anatomical; // It marker is a anatomical marker
        int m_id;

};

#endif // S2M_IMU_H
