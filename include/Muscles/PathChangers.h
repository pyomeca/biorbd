#ifndef BIORBD_MUSCLES_PATH_CHANGERS_H
#define BIORBD_MUSCLES_PATH_CHANGERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Node3d;
}

namespace muscles {
    ///
    /// \brief Class Path Changers
    ///
class BIORBD_API PathChangers
{
public:
    ///
    /// \brief Construct path changers
    /// 
    PathChangers();
    
    /// 
    /// \brief Deep copy of path changers
    /// \return A deep copy of path changers
    ///
    biorbd::muscles::PathChangers DeepCopy() const;
    /// 
    /// \brief Deep copy of path changers from another path changers
    /// \param other THe path changers to copy
    ///    
    void DeepCopy(const biorbd::muscles::PathChangers& other);

    ///
    /// \brief Add a wrapping or a via point
    /// \param point The wrapping or via point to add
    ///
    void addPathChanger(biorbd::utils::Node3d&); 

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
    biorbd::utils::Node3d& object(unsigned int  idx);
    ///
    /// \brief Return an object at a specific index
    /// \param idx Index of the object
    /// \return The object at a specific index
    ///
    const biorbd::utils::Node3d& object(unsigned int  idx) const; 

protected:
    std::shared_ptr<std::vector<biorbd::utils::Node3d>> m_obj; ///<Tbale of pointers on the objects
    std::shared_ptr<unsigned int> m_nbWraps; ///< Number of wraps
    std::shared_ptr<unsigned int> m_nbVia; ///< TODO: ?
    std::shared_ptr<unsigned int> m_totalObjects; ///< Total objects

};

}}

#endif // BIORBD_MUSCLES_PATH_CHANGERS_H
