#ifndef BIORBD_UTILS_PATH_H
#define BIORBD_UTILS_PATH_H

#include <memory>
#ifdef _WIN32
#include <string>
#endif
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
///
/// \brief Class Path
///
class BIORBD_API Path
{
public: 
    ///
    /// \brief Construct path
    ///
    Path();
    ///
    /// \brief Construct path
    /// \param c The path (char)
    /// 
    Path(
            const char *c);
    ///
    /// \brief Construct path
    /// \param s The path (string)
    Path(
            const biorbd::utils::String &s);
    ///
    /// \brief Construct path
    /// \param c The path (basic string char)
    ///
    Path(
            const std::basic_string<char> &c);
    
    ///
    /// \brief Deep copy of path
    /// \return A deep copy of path
    ///
    biorbd::utils::Path DeepCopy() const;
    ///
    /// \brief Deep copy of path from another path
    /// \param other The path to copy
    ///
    void DeepCopy(
            const biorbd::utils::Path& other);


    ///
    /// \brief Parse a path in folder, filename and extension
    /// \param path The path to parse
    /// \param folder The folder
    /// \param filename The file name
    /// \param ext The extension
    ///
    static void parseFileName(
            const biorbd::utils::String& path,
            biorbd::utils::String& folder,
            biorbd::utils::String& filename,
            biorbd::utils::String& ext);

    // Tests sur les fichiers
    /// 
    /// \brief If file exist
    /// \return True or False
    ///
    bool isFileExist() const;
    
    ///
    /// \brief If file exist
    /// \param path The path of the file
    /// \return True or False
    ///
    static bool isFileExist(
            const biorbd::utils::Path&path);

    ///
    /// \brief If file exist
    /// \param path The path of the file
    /// \return True or False
    ///
    static bool isFileExist(
            const biorbd::utils::String&path);

    ///
    /// \brief If file is readable
    /// \return True or False
    ///
    bool isFileReadable() const;

    ///
    /// \brief If folder exists
    /// \return True or False
    ///
    bool isFolderExist() const;
    ///
    /// \brief If folder exists
    /// \param path Path of the folder
    /// \return True or False
    ///
    static bool isFolderExist(
            const biorbd::utils::Path&path);
    ///
    /// \brief If folder exists
    /// \param path Path of the folder
    /// \return True or False
    ///
    static bool isFolderExist(
            const biorbd::utils::String&path);
    ///
    /// \brief To create folder
    ///
    void createFolder() const;
    ///
    /// /brief Return current directory
    /// \return Current directory
    ///
    static biorbd::utils::String currentDir();
    ///
    /// \brief Return relative path to current working directory
    /// \return Relative path to curent working directory
    ///
    biorbd::utils::String relativePath() const;
    ///
    /// \brief Return relative path 
    /// \param relativeTo Relative to that path
    /// \return Relative path
    ///
    biorbd::utils::String relativePath(
            const biorbd::utils::String &relativeTo) const; 
    ///
    /// \brief Return relative path 
    /// \param path The path
    /// \param relativeTo Relative to that path
    /// \return Relative path
    ///
    static biorbd::utils::String relativePath(
            const biorbd::utils::Path &path,
            const biorbd::utils::String &relativeTo); 
    ///
    /// \brief Return the absolute folder relative to root
    /// \param path The path
    /// \return The absolute folder
    ///
    static biorbd::utils::String absoluteFolder(
            const biorbd::utils::Path &path); 
    ///
    /// \brief Return the absolute folder relative to root
    /// \return The absolute folder relative to root
    ///
    biorbd::utils::String absoluteFolder() const;
    ///
    /// \brief Return the absolute path relative to root
    /// \return The absolute path relative to root
    ///
    biorbd::utils::String absolutePath() const; 

    ///
    /// \brief Return the path to Unix format
    /// \param path The path
    /// \return The path in Unix format
    ///
    static biorbd::utils::String toUnixFormat(
            const biorbd::utils::String& path);

    ///
    /// \brief Return the path to Windows format
    /// \param path The path
    /// \return The path in Windows format
    ///
    static biorbd::utils::String toWindowsFormat(
            const biorbd::utils::String& path);
    // Accessor
    ///
    /// \brief Return original path
    /// \return Original path
    ///
    const biorbd::utils::String& originalPath() const;
    ///
    /// \brief Return folder
    /// \return Folder
    ///
    const biorbd::utils::String& folder() const;
    ///
    /// \brief Return filename
    /// \return Filename
    ///
    const biorbd::utils::String& filename() const;
    ///
    /// \brief Set the filename
    /// \param name Filename
    ///
    void setFilename(
            const biorbd::utils::String& name);
    ///
    /// \brief Return the extension
    /// \return The extension
    /// 
    const biorbd::utils::String &extension() const;
    ///
    /// \brief Set the extension
    /// \param ext The extension
    ///
    void setExtension(
            const biorbd::utils::String& ext);

protected:
    ///
    /// \brief Set if folder is absolute
    ///
    void setIsFolderAbsolute();

    std::shared_ptr<biorbd::utils::String> m_originalPath; ///< The original path
    std::shared_ptr<biorbd::utils::String> m_folder; ///< The folder
    std::shared_ptr<bool> m_isFolderAbsolute; ///< If folder is absolute
    std::shared_ptr<biorbd::utils::String> m_filename; ///< The filename
    std::shared_ptr<biorbd::utils::String> m_extension; ///< The extension
};

}}

#endif // BIORBD_UTILS_PATH_H
