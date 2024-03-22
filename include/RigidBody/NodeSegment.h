#ifndef BIORBD_RIGIDBODY_NODE_SEGMENT_H
#define BIORBD_RIGIDBODY_NODE_SEGMENT_H

#include <vector>
#include "biorbdConfig.h"
#include "Utils/Vector3d.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
}

namespace rigidbody
{

///
/// \brief A point attached to a segment, generally speaking a skin marker
///
class BIORBD_API NodeSegment : public utils::Vector3d
{
public:
    ///
    /// \brief Construct a segment node
    ///
    NodeSegment();

    ///
    /// \brief Construct a segment node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    ///
    NodeSegment(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z);

    ///
    /// \brief Construct a segment node from another node
    /// \param other The other node
    ///
    NodeSegment(
        const utils::Vector3d& other);

    ///
    /// \brief Construct a segment node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param isTechnical If the node is technical
    /// \param isAnatomical If the node is anatomical
    /// \param axesToRemove The axes to remove
    /// \param parentID The index of the parent segment
    ///
    NodeSegment(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z,
        const utils::String& name,
        const utils::String& parentName,
        bool isTechnical,
        bool isAnatomical,
        const utils::String& axesToRemove,
        int parentID);

    ///
    /// \brief Construct a segment node
    /// \param node The position of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param isTechnical If the node is technical
    /// \param isAnatomical If the node is anatomical
    /// \param axesToRemove The axes to remove
    /// \param parentID The index of the parent segment
    ///
    NodeSegment(
        const utils::Vector3d& node,
        const utils::String& name,
        const utils::String& parentName,
        bool isTechnical,
        bool isAnatomical,
        const utils::String& axesToRemove,
        int parentID);

    ///
    /// \brief Deep copy of the segment node
    /// \return A deep copy of the segment node
    ///
    NodeSegment DeepCopy() const;

    ///
    /// \brief Deep copy of the segment node
    /// \param other The segment node to copy
    ///
    void DeepCopy(const NodeSegment& other);

    // Get and Set

    /// 
    /// \brief Set internal values of the node without changing other parameters
    /// \param values The new values
    /// 
    void setValues(const NodeSegment& values);

    ///
    /// \brief Return if node is technical
    /// \return If node is technical
    ///
    bool isTechnical() const;

    ///
    /// \brief Return if node is anatomical
    /// \return If node is anatomical
    ///
    bool isAnatomical() const;

    ///
    /// \brief Return the parent index
    /// \return The parent index
    ///
    int parentId() const;

    ///
    /// \brief Accessor to axes to remove
    /// \return The axes to remove
    ///
    const std::vector<bool>& axesToRemove() const;

    ///
    /// \brief A vector of bool which is the opposite of axesToRemove
    /// \return The axes to keep
    ///
    const std::vector<bool> axes() const;

    ///
    /// \brief To remove axis
    /// \return Projected position of the node when removing speficic axes speficied using addAxesToRemove
    ///
    NodeSegment removeAxes() const;

    ///
    /// \brief Check if axis is removed
    /// \return If axis is removed or not
    ///
    bool isAxisRemoved(size_t) const;

    ///
    /// \brief Check if axis is kept
    /// \return If axis is kept or not
    ///
    bool isAxisKept(size_t) const;

    ///
    /// \brief Get all the indices of the axes to keep (X=0, Y=1, Z=2)
    /// 
    std::vector<int> availableAxesIndices() const;

    ///
    /// \brief Add an axis to remove
    /// \param axisNumber The axis number to remove (x = 0, y = 1 and z = 2)
    ///
    void addAxesToRemove(size_t axisNumber);

    ///
    /// \brief Add axis to remove
    /// \param axis The name of the axis to remove ("x", "y" or "z")
    ///
    void addAxesToRemove(const utils::String& axis);

#ifndef SWIG
    ///
    /// \brief Add multiple axes to remove
    /// \param axes The multiples axes numbers to remove (x = 0, y = 1 and z = 2)
    ///
    void addAxesToRemove(const std::vector<size_t>& axes);
#endif

    ///
    /// \brief Add multiple axes to remove
    /// \param axes The multiples axes names to remove ("x", "y" or "z")
    ///
    void addAxesToRemove(const std::vector<utils::String>& axes);

    ///
    /// \brief Return the axes to removed
    /// \return The axes to removed
    ///
    utils::String axesToRemoveAsString() const;

    ///
    /// \brief Return the number of axes to remove
    /// \return The number of axes to remove
    ///
    int nbAxesToRemove() const;


#ifndef SWIG
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allows for operator= with other Vector3d
    /// \param other The other vector
    ///
    template<typename OtherDerived>
    NodeSegment & operator=(const Eigen::MatrixBase
            <OtherDerived>& other)
    {
        this->utils::Vector3d::operator=(other);
        return *this;
    }
#endif
#endif

protected:
    ///
    /// \brief Set the type of the segment node
    ///
    void setType();

    
    std::shared_ptr<std::vector<bool>> m_axesRemoved; ///< The axes to remove
    std::shared_ptr<int> m_nbAxesToRemove; ///< Removed one of multiple axes (1 axis : project on a plan, 2 axes : project on the 3rd axis, 3 axes : return the position of the parent reference)
    std::shared_ptr<bool> m_technical; ///< If a marker is a technical marker
    std::shared_ptr<bool> m_anatomical; ///< It marker is a anatomical marker
    std::shared_ptr<int> m_id; ///< The parent identification

};

}
}

#endif // BIORBD_RIGIDBODY_NODE_SEGMENT_H
