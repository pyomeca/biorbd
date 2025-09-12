#define BIORBD_API_EXPORTS
#include "Utils/CasadiExpand.h"

using namespace BIORBD_NAMESPACE;

#ifdef USE_SMOOTH_IF_ELSE
casadi::MX utils::lt(const casadi::MX& x, const casadi::MX& y) { return x - y; }

casadi::MX utils::le(const casadi::MX& x, const casadi::MX& y) {
  return utils::lt(x, y);
}

casadi::MX utils::gt(const casadi::MX& x, const casadi::MX& y) {
  return utils::lt(y, x);
}

casadi::MX utils::ge(const casadi::MX& x, const casadi::MX& y) {
  return utils::le(y, x);
}

casadi::MX utils::if_else(
    const casadi::MX& cond,
    const casadi::MX& if_true,
    const casadi::MX& if_false,
    double b) {
  return if_true +
         (if_false - if_true) * (0.5 + 0.5 * casadi::MX::tanh(b * cond));
}

casadi::MX utils::if_else_zero(
    const casadi::MX& cond,
    const casadi::MX& if_true,
    double b) {
  return utils::if_else(cond, if_true, 0, b);
}
#endif
