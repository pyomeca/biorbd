#ifndef BIORBD_UTILS_NODE_H
#define BIORBD_UTILS_NODE_H

#include "biorbdConfig.h"

#include <memory>

#include "Utils/UtilsEnum.h"

namespace BIORBD_NAMESPACE {
namespace utils {
class String;

///
/// \brief A node is an abstract element which is assigned to a parent
///
class BIORBD_API Node {
 public:
  ///
  /// \brief Construct Node
  ///
  Node();

  ///
  /// \brief Construct Node from another node
  /// \param other The other node
  ///
  Node(const Node &other);

  ///
  /// \brief Construct Node
  /// \param name Name of the node
  ///
  Node(const String &name);

  ///
  /// \brief Construct Node
  /// \param name Name of the node
  /// \param parentName Name of the parent of the node
  ///
  Node(const String &name, const String &parentName);

  ///
  /// \brief Destroy class properly
  ///
  virtual ~Node();

  ///
  /// \brief Deep copy of the node in another node
  /// \param other The node to copy
  ///
  void DeepCopy(const Node &other);

  ///
  /// \brief Set the name of the node
  /// \param name Name to set
  ///
  void setName(const String &name);

  ///
  /// \brief Return the name of the node
  ///
  const String &name() const;

  ///
  /// \brief Return the parent name of the node
  ///
  const String &parent() const;

  ///
  /// \brief Set the parent name of the node
  /// \param name The name of the parent
  ///
  void setParent(const String &name);

  ///
  /// \brief Return the type of node
  ///
  NODE_TYPE typeOfNode() const;

 protected:
  ///
  /// \brief To set the type
  ///
  virtual void setType() = 0;

  std::shared_ptr<String> m_name;           ///< The name of the node
  std::shared_ptr<String> m_parentName;     ///< The parent name of the node
  std::shared_ptr<NODE_TYPE> m_typeOfNode;  ///< The type of the node
};

}  // namespace utils
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_UTILS_NODE_H
