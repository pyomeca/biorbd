#ifndef S2MMUSCLEHILLTYPEMAXIME_H
#define S2MMUSCLEHILLTYPEMAXIME_H
    #include "biorbdConfig.h"
    #include "s2mMuscleHillType.h"



class BIORBD_API s2mMuscleHillTypeMaxime : public s2mMuscleHillType
{
    public:
    s2mMuscleHillTypeMaxime(const s2mString& s= "") : s2mMuscleHillType(s), m_x0(0), m_x1(0){setType();}
        s2mMuscleHillTypeMaxime(const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual());
        s2mMuscleHillTypeMaxime(const s2mString& n,
                                const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual());
        ~s2mMuscleHillTypeMaxime(){}

        void x0(const double x){m_x0 = x;}
        double x0(){return m_x0;}
        void x1(const double x){m_x1 = x;}
        double x1(){return m_x1;}

        void muscleGain(const double& val){m_muscleGain = val;}
        double muscleGain(){return m_muscleGain;}

        virtual void forceIsoMax(double val);

        // Réimplémentation de caract pour retourner celui souhaité
        virtual const s2mMuscleCaracteristics& caract() const { return m_caractMaxime; }
        virtual void setCaract(const s2mMuscleCaracteristics &val) { m_caractMaxime = val; }
    protected:
        class s2mMuscleCaracteristicsMaxime : public s2mMuscleCaracteristics{
            public:
                s2mMuscleCaracteristicsMaxime(const s2mMuscleCaracteristics&);

                s2mMuscleCaracteristicsMaxime(const double &optLength = 0,
                                        const double &fmax = 0,
                                        const double &PCSA = 0,
                                        const double &tendonSlackLength = 0,
                                        const double &pennAngle = 0,
                                        const s2mMuscleStateMax *stateMax = nullptr,
                                        const double tauAct = 0.01,
                                        const double tauDeact = 0.04,
                                        const double &minAct =.01
                                        );
                void setForceIsoMax(const double &val);
                void bruteSetForceIsoMax(const double &val);
                bool isForceMaxSet() const {return m_isForceMaxSet;}
                double optimalLength() const {
                    return  m_optimalLength*cos(m_pennationAngle)+m_tendonSlackLength;}

            private:
                bool m_isForceMaxSet;
        };
        virtual const s2mMuscleCaracteristicsMaxime& caractMaxime() const { return m_caractMaxime; }


        // Réimplémentation du calcul de la force
        virtual s2mMuscleStateActual excitationNorm(s2mMuscleStateActual EMG);
        void setType(){m_type = "HillMax";}
        virtual void computeForce(const s2mMuscleStateActual &EMG); // Calcul des forces

        double m_x0;
        double m_x1;

        double m_muscleGain;
        s2mMuscleCaracteristicsMaxime m_caractMaxime;
    private:

};

#endif // S2MMUSCLEHILLTYPEMAXIME_H
