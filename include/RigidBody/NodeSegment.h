#ifndef BIORBD_RIGIDBODY_NODE_SEGMENT_H
#define BIORBD_RIGIDBODY_NODE_SEGMENT_H

#include <vector>
#include "biorbdConfig.h"
#include "Utils/Vector3d.h"

namespace biorbd {
namespace utils {
class String;
}

namespace rigidbody {

///
/// \brief Class NodeBone
///
class BIORBD_API NodeSegment : public biorbd::utils::Vector3d
{ 
public:
    ///
    /// \brief Construct a segment node
    ///
    NodeSegment();
    
    ///
    /// \brief Construct a segment node
    /// \param x Position of the node on the x axis
    /// \param y Position of the node on the y axis
    /// \param z Position of the node on the z axis
    ///
    NodeSegment(
            double x,
            double y,
            double z);

    ///
    /// \brief Construct a segment node from another node
    /// \param other The other node
    ///
    NodeSegment(const biorbd::utils::Vector3d& other);
    
    ///
    /// \brief Construct a segment node
    /// \param x Position of the node on the x axis
    /// \param y Position of the node on the y axis
    /// \param z Position of the node on the z axis
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param isTechnical If the node is technical
    /// \param isAnatomical If the node is anatomical
    /// \param axesToRemove The axis to remove
    /// \param parentID The identification number of the parent
    ///
    NodeSegment(
            double x,
            double y,
            double z,
            const biorbd::utils::String& name, 
            const biorbd::utils::String& parentName, 
            bool isTechnical, 
            bool isAnatomical, 
            const biorbd::utils::String& axesToRemove, 
            int parentID); 

    ///
    /// \brief Construct a segment node
    /// \param node The position of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param isTechnical If the node is technical
    /// \param isAnatomical If the node is anatomical
    /// \param axesToRemove The axis to remove
    /// \param parentID The identification number of the parent
    ///

    NodeSegment(
            const biorbd::utils::Vector3d& node, 
            const biorbd::utils::String& name, 
            const biorbd::utils::String& parentName, 
            bool isTechnical, 
            bool isAnatomical, 
            const biorbd::utils::String& axesToRemove, 
            int parentID); 

    ///
    /// \brief Deep copy of the segment node
    /// \return A deep copy of the segment node
    ///
    biorbd::rigidbody::NodeSegment DeepCopy() const;

    ///
    /// \brief Deep copy of the segment node
    /// \param other The segment node to copy
    ///
    void DeepCopy(const biorbd::rigidbody::NodeSegment& other);

    // Get and Set

    ///
    /// \brief Check if node is technical
    /// \return If node is technical
    ///
    bool isTechnical() const;

    ///
    /// \brief Check if node is anatomical
    /// \return If node is anatomical
    ///
    bool isAnatomical() const;

    ///
    /// \brief Return the parent identification
    /// \return The parent identification
    ///
    int parentId() const;

    ///
    /// \brief To remove axis
    /// \return Position variables without the axis TODO
    ///
    NodeSegment removeAxes() const;

    ///
    /// \brief Check if axis is removed
    /// \return If axis is removed or not
    ///
    bool isAxisRemoved(unsigned int) const;

    /// 
    /// \brief Check if axis is kept
    /// \return If axis is kept or not
    ///
    bool isAxisKept(unsigned int) const;

    ///
    /// \brief Add axis to remove
    /// \param axisNumber The axis number to remove (must be 0 (x), 1 (y) or 2 (z))
    ///
    void addAxesToRemove(unsigned int axisNumber);

    ///
    /// \brief Add axis to remove
    /// \param axis The name of the axis to remove (x, y or z)
    ///
    void addAxesToRemove(const biorbd::utils::String& axis); 

    ///
    /// \brief Add multiple axes to remove
    /// \param axes The multiples axes numbers to remove (must be 0 (x), 1 (y) or 2 (z))
    ///
    void addAxesToRemove(const std::vector<unsigned int>& axes); 

    ///
    /// \brief Add multiple axes to remove
    /// \param axes The multiples axes names to remove (x,y,z)
    ///
    void addAxesToRemove(const std::vector<biorbd::utils::String>& axes); 

    ///
    /// \brief Check which axes have been removed
    /// \return The axes that have been removed
    ///
    biorbd::utils::String axesToRemove();

    ///
    /// \brief Return the number of axes to remove
    /// \return The number of axes to remove
    ///
    int nbAxesToRemove() const; 

    ///
    /// \brief TODO
    /// \param other TODO
    ///
    template<typename OtherDerived>
        biorbd::rigidbody::NodeSegment & operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::Vector3d::operator=(other);
            return *this;
        }

protected:
    ///
    /// \brief Set the type of the segment node
    ///
    void setType();
    std::shared_ptr<std::vector<bool>> m_axesRemoved; ///< Projection matrix
    std::shared_ptr<int> m_nbAxesToRemove; ///< Removed one of multiple axes (1 axis : project on a plan, 2 axes : project on the 3rd axis, 3 axes : return the position of the parent reference)
    std::shared_ptr<bool> m_technical; ///< If a marker is a technical marker
    std::shared_ptr<bool> m_anatomical; ///< It marker is a anatomical marker
    std::shared_ptr<int> m_id; ///< The parent identification

};

}}

#endif // BIORBD_RIGIDBODY_NODE_SEGMENT_H
