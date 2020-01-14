#ifndef BIORBD_UTILS_GENERALIZED_COORDINATE_RANGE_H
#define BIORBD_UTILS_GENERALIZED_COORDINATE_RANGE_H

#include "biorbdConfig.h"

#include <memory>

namespace biorbd {
namespace rigidbody {

///
/// \brief Class GeneralizedCoordinateRange
///
class BIORBD_API GeneralizedCoordinateRange
{
public:

    ///
    /// \brief Construct generalized coordinates range
    /// 
    GeneralizedCoordinateRange(
            double min = -M_PI,
            double max = M_PI);

    ///
    /// \brief Deep copy of the GeneralizedCoordinateRange
    /// \return Deep copy of the GeneralizedCoordinateRange
    ///
    biorbd::rigidbody::GeneralizedCoordinateRange DeepCopy() const;

    ///
    /// \brief Deep copy of GeneralizedCoordinateRange
    /// \param other The GeneralizedCoordinateRange to copy
    ///
    void DeepCopy(const biorbd::rigidbody::GeneralizedCoordinateRange& other);

    ///
    /// \brief Set a new value for the minimum
    /// \param min The value to set to
    ///
    void setMin(double min);

    ///
    /// \brief Return the minimum value
    /// \return The minimum value
    ///
    double min() const;

    ///
    /// \brief Set a new value for the maximal
    /// \param min The value to set to
    ///
    void setMax(double max);

    ///
    /// \brief Return the maximum value
    /// \return The minimum value
    ///
    double max() const;

protected:
    std::shared_ptr<double> m_min;
    std::shared_ptr<double> m_max;
};

}}

#endif // BIORBD_UTILS_GENERALIZED_COORDINATE_RANGE_H
