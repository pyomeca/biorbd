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
    Equation(const char *string);
    Equation(const biorbd::utils::String &string);
    Equation(const std::basic_string<char> &string);

    static std::vector<biorbd::utils::Equation> splitIntoEquation(
            biorbd::utils::Equation wholeEq,
            const std::map<biorbd::utils::Equation, double>& variables);
    static double resolveEquation(
            std::vector<biorbd::utils::Equation> wholeEq);
    static double resolveEquation(
            biorbd::utils::Equation wholeEq);
    static double resolveEquation(
            biorbd::utils::Equation wholeEq,
            const std::map<biorbd::utils::Equation, double>& variables);
    static void replaceCste(
            std::vector<biorbd::utils::Equation> &eq);
    static void replaceVar(
            Equation &eq,
            const std::map<biorbd::utils::Equation, double>& variables);

protected:
    static double resolveEquation(
            std::vector<biorbd::utils::Equation> eq,
            unsigned int math);
    static std::vector<biorbd::utils::Equation> prepareMathSymbols();
};

}}

#endif // BIORBD_UTILS_EQUATION_H
