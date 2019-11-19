#ifndef BIORBD_UTILS_EQUATION_H
#define BIORBD_UTILS_EQUATION_H

#include <vector>
#include <map>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace utils {
#define PI 3.141592653589793
///
/// \brief Equation
///
class BIORBD_API Equation : public biorbd::utils::String
{
public:
    ///
    /// \brief Construct equation
    ///
    Equation();
    ///
    /// \brief Construct equation 
    /// \param string Name of the equation TODO:
    ///
    Equation(const char *string);
    ///
    /// \brief Construct equation 
    /// \param string Name of the equation TODO:
    ///
    Equation(const biorbd::utils::String &string);
    ///
    /// \brief Construct equation 
    /// \param string Name of the equation TODO:
    ///
    Equation(const std::basic_string<char> &string);

    ///
    /// \brief Split equation into smaller parts in a vector (string)
    /// \param wholeEq Whole equation to split
    /// \param variables The variables in the equation
    /// \return The equation
    ///
    static std::vector<biorbd::utils::Equation> splitIntoEquation(
            biorbd::utils::Equation wholeEq,
            const std::map<biorbd::utils::Equation, double>& variables);

    ///
    /// \brief Return the resolve equation
    /// \param wholeEq Whole equation to split
    /// \return The resolved equation
    ///
    static double resolveEquation(
            std::vector<biorbd::utils::Equation> wholeEq);

    ///
    /// \brief Return the resolve equation
    /// \param wholeEq Whole equation to split
    /// \return The resolved equation
    ///
    static double resolveEquation(
            biorbd::utils::Equation wholeEq);
    ///
    /// \brief Return the resolve equation
    /// \param wholeEq Whole equation to split
    /// \param variables The variables in the equation
    /// \return The resolved equation
    ///
    static double resolveEquation(
            biorbd::utils::Equation wholeEq,
            const std::map<biorbd::utils::Equation, double>& variables);

    ///
    /// \brief Replace the constants by a number
    /// \param eq The equation
    ///
    static void replaceCste(
            std::vector<biorbd::utils::Equation> &eq);

    ///
    /// \brief Replace variable 
    /// \param eq The equation
    /// \param variables The variables to replace? TODO:
    ///
    static void replaceVar(
            Equation &eq,
            const std::map<biorbd::utils::Equation, double>& variables);

protected:
    ///
    /// \brief Resolve the equation
    /// \param eq The equation to resolve
    /// \param math TODO:
    /// 
    static double resolveEquation(
            std::vector<biorbd::utils::Equation> eq,
            unsigned int math);
    ///
    /// \brief Prepare the mathematical symbols
    /// \return The math symbols
    ///
    static std::vector<biorbd::utils::Equation> prepareMathSymbols();
};

}}

#endif // BIORBD_UTILS_EQUATION_H
