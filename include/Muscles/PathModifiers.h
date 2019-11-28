#ifndef BIORBD_MUSCLES_PATH_MODIFIERS_H
#define BIORBD_MUSCLES_PATH_MODIFIERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Vector3d;
}

namespace muscles {
///
/// \brief Holder of all the path modifiers of a muscle
///
class BIORBD_API PathModifiers
{
public:
    ///
    /// \brief Construct path changers
    /// 
    PathModifiers();
    
    /// 
    /// \brief Deep copy of path changers
    /// \return A deep copy of path changers
    ///
    biorbd::muscles::PathModifiers DeepCopy() const;
    /// 
    /// \brief Deep copy of path changers from another path changers
    /// \param other THe path changers to copy
    ///    
    void DeepCopy(const biorbd::muscles::PathModifiers& other);

    ///
    /// \brief Add a wrapping or a via point
    /// \param point The wrapping or via point to add
    ///
    void addPathChanger(biorbd::utils::Vector3d&point); 

    // Set and get
    ///
    /// \brief Return the total number of wrapping objects
    /// \return The total number of wrapping objects
    ///
    unsigned int nbWraps() const;

    ///
    /// \brief Return the total number of via points
    /// \return The total number of via points
    ///
    unsigned int nbVia() const;

    ///
    /// \brief Return the total number of objects
    /// \return The total number of objects
    ///
    unsigned int nbObjects() const;

    ///
    /// \brief Return an object at a specific index
    /// \param idx Index of the object
    /// \return The object at a specific index
    ///
    biorbd::utils::Vector3d& object(unsigned int  idx);
    ///
    /// \brief Return an object at a specific index
    /// \param idx Index of the object
    /// \return The object at a specific index
    ///
    const biorbd::utils::Vector3d& object(unsigned int  idx) const; 

protected:
    std::shared_ptr<std::vector<biorbd::utils::Vector3d>> m_obj; ///<Tbale of pointers on the objects
    std::shared_ptr<unsigned int> m_nbWraps; ///< Number of wraps
    std::shared_ptr<unsigned int> m_nbVia; ///< TODO: ?
    std::shared_ptr<unsigned int> m_totalObjects; ///< Total objects

};

}}

#endif // BIORBD_MUSCLES_PATH_MODIFIERS_H
