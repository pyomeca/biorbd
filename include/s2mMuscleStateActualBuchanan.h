#ifndef S2MMUSCLESTATEACTUALBUCHANAN_H
#define S2MMUSCLESTATEACTUALBUCHANAN_H
#include "s2mMuscleStateActual.h"

class s2mMuscleStateActualBuchanan : public s2mMuscleStateActual
{
    public:
        s2mMuscleStateActualBuchanan(const double &e = 0, const double &a = 0);
        ~s2mMuscleStateActualBuchanan();

        virtual double timeDerivativeActivation(const s2mMuscleCaracteristics &c, const bool alreadyNormalized);
        void shapeFactor(double m_shape_factor);
        double shapeFactor();

    protected:
        double m_shape_factor; //Buchanan2004, le 22 mars 2018
};

#endif // S2MMUSCLESTATEACTUALBUCHANAN_H
