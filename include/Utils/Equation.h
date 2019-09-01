#ifndef BIORBD_UTILS_EQUATION_H
#define BIORBD_UTILS_EQUATION_H

#include <vector>
#include <map>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace utils {
#define PI 3.141592653589793

class BIORBD_API Equation : public biorbd::utils::String
{
public:
    Equation();
    Equation(const char *c);
    Equation(const biorbd::utils::String &s);
    Equation(const std::basic_string<char> &c);
    biorbd::utils::Equation DeepCopy() const;
    void DeepCopy(const biorbd::utils::Equation& other);

    static std::vector<biorbd::utils::Equation> splitIntoEquation(
            biorbd::utils::Equation, const std::map<biorbd::utils::Equation, double>&);
    static double resolveEquation(std::vector<biorbd::utils::Equation>);
    static double resolveEquation(biorbd::utils::Equation);
    static double resolveEquation(biorbd::utils::Equation, const std::map<biorbd::utils::Equation, double>&);
    static void replaceCste(std::vector<biorbd::utils::Equation> &eq);
    static void replaceVar(
            std::vector<biorbd::utils::Equation> &eq,
            const std::map<biorbd::utils::Equation, double>&);

protected:
    static double resolveEquation(
            std::vector<biorbd::utils::Equation> eq,
            unsigned int math);
    static std::vector<biorbd::utils::Equation> prepareMathSymbols();
};

}}

#endif // BIORBD_UTILS_EQUATION_H
