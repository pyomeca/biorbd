#ifndef BIORBD_UTILS_TIME_H
#define BIORBD_UTILS_TIME_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
///
/// \brief Class Time
 ///
class BIORBD_API Time
{
public:
    ///
    /// \brief Construct time
    ///
    Time();
    ///
    /// \brief Construct time
    /// \param timeStep The time step
    /// \param nbSteps The number of steps
    ///
    Time(
            double timeStep,
            unsigned int nbSteps);
    ///
    /// \brief Return the time at index t
    /// \param t Index 
    /// \return The time at index t
    ///
    double time(unsigned int t); 
protected:
    std::vector<double> m_time; ///< The time

};

}}

#endif // BIORBD_UTILS_TIME_H
