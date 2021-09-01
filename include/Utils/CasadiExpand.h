#ifndef BIORBD_UTILS_CASADI_EXPAND_H
#define BIORBD_UTILS_CASADI_EXPAND_H

#include <map>
#include "biorbdConfig.h"

#include "casadi.hpp"

namespace BIORBD_NAMESPACE
{
namespace utils
{
#ifdef USE_SMOOTH_IF_ELSE
///
/// \brief Lesser than that works with the if_else of CasadiExpand
/// \param x First element
/// \param y Second element
/// \return The lesser than element
///
casadi::MX lt(
        const casadi::MX& x,
        const casadi::MX& y);

///
/// \brief Lesser or equal than that works with the if_else of CasadiExpand.
/// This is strictly similar to lt
/// \param x First element
/// \param y Second element
/// \return The lesser than element
///
casadi::MX le(
        const casadi::MX& x,
        const casadi::MX& y);

///
/// \brief Greater than that works with the if_else of CasadiExpand
/// \param x First element
/// \param y Second element
/// \return The greater than element
///
casadi::MX gt(
        const casadi::MX& x,
        const casadi::MX& y);

///
/// \brief Greater or equal than that works with the if_else of CasadiExpand.
/// This is strictly similar to lt
/// \param x First element
/// \param y Second element
/// \return The greater than element
///
casadi::MX ge(
        const casadi::MX& x,
        const casadi::MX& y);

///
/// \brief A non-branching and continuous if_else based on tanh. This is not
/// strictly equal to if_else when the values are near each other. The b parameter
/// can be adjusted to increase the rate of change
/// \param cond The condition (lt/le/gt/ge)
/// \param if_true The value to return if true
/// \param if_false The value to return if false
/// \param b The slope parameter
/// \return The value of the if_else function
///
casadi::MX if_else(
        const casadi::MX& cond,
        const casadi::MX& if_true,
        const casadi::MX& if_false,
        double b = 10000);

///
/// \brief A non-branching and continuous if_else_zero based on tanh. This is not
/// strictly equal to if_else_zero when the values are near each other. The b parameter
/// can be adjusted to increase the rate of change
/// \param cond The condition (lt/le/gt/ge)
/// \param if_true The value to return if true
/// \param if_false The value to return if false
/// \param b The slope parameter
/// \return The value of the if_else function
///
casadi::MX if_else_zero(
        const casadi::MX& cond,
        const casadi::MX& if_true,
        double b = 10000);
#endif

}
}

#endif // BIORBD_UTILS_CASADI_EXPAND_H

