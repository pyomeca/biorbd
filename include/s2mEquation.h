#ifndef S2MEQUATION_H
#define S2MEQUATION_H

#include "biorbdConfig.h"
#include "s2mString.h"
#include "s2mError.h"
#define PI 3.141592653589793

class BIORBD_API s2mEquation : public s2mString
{
public:
    s2mEquation();
    s2mEquation(const char *c);
    s2mEquation(const s2mString &s);
    s2mEquation(const std::basic_string<char> &c);

    static std::vector<s2mEquation> splitIntoEquation(s2mEquation, const std::map<s2mEquation, double>&);
    static double resolveEquation(std::vector<s2mEquation>);
    static double resolveEquation(s2mEquation);
    static double resolveEquation(s2mEquation, const std::map<s2mEquation, double>&);
    static void replaceCste(std::vector<s2mEquation> &eq);
    static void replaceVar(std::vector<s2mEquation> &eq, const std::map<s2mEquation, double>&);

protected:
    static double resolveEquation(std::vector<s2mEquation> eq, unsigned int math);
    static std::vector<s2mEquation> prepareMathSymbols();
};


#endif // S2MEQUATION_H
