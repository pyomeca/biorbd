#ifndef BIORBD_UTILS_TIME_H
#define BIORBD_UTILS_TIME_H

#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {

class BIORBD_API Time
{
public:
    Time(
            double timeStep,
            unsigned int nbSteps);

    double time(unsigned int t); // Return time at index t
protected:
    std::vector<double> m_time;
    unsigned int m_nbSteps;

};

}}

#endif // BIORBD_UTILS_TIME_H
