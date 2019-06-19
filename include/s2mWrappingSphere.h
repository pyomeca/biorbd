#ifndef S2MWRAPPINGSPHERE_H
#define S2MWRAPPINGSPHERE_H
    #include "biorbdConfig.h"
    #include "s2mWrappingObject.h"

class BIORBD_API s2mWrappingSphere : public s2mWrappingObject
{
    public:
        s2mWrappingSphere(const double &dia = 0, // Diametre de la sphere
                          const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
                          const s2mString &name = "",  // Nom du noeud
                          const s2mString &parentName = "");
        ~s2mWrappingSphere();
        s2mAttitude RT(s2mJoints &, const s2mGenCoord& , const bool & = true) {return s2mAttitude();}
        virtual void wrapPoints(const s2mAttitude&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) {} // Premier et dernier points musculaire
        virtual void wrapPoints(s2mJoints&, const s2mGenCoord&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) {} // Premier et dernier points musculaire
        virtual void wrapPoints(s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) {} // Premier et dernier points musculaire

        // Get and set
        double size() const { return m_dia; }
        void setSize(const double &val) { m_dia = val; }
//        void setSize() {m_dia = 1;}
        s2mString forme() const {return "Sphere";}
    protected:
        double m_dia;
    private:
};

#endif // S2MWRAPPINGSPHERE_H
