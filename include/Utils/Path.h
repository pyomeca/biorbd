#ifndef BIORBD_UTILS_PATH_H
#define BIORBD_UTILS_PATH_H

#include <memory>
#ifdef _WIN32
    #include <string>
#endif
#include "biorbdConfig.h"

namespace biorbd
{
namespace BIORBD_MATH_NAMESPACE
{
namespace utils
{
class String;
///
/// \brief Collection of methods to manipulate path
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
    /// \param path The path in char format
    ///
    Path(
        const char* path);

    ///
    /// \brief Construct path
    /// \param path The path in string format
    ///
    Path(
        const String& path);

    ///
    /// \brief Construct path
    /// \param path The path in vector char format
    ///
    Path(
        const std::basic_string<char>& path);

    ///
    /// \brief Deep copy of path
    /// \return A deep copy of path
    ///
    Path DeepCopy() const;

    ///
    /// \brief Deep copy of path from another path
    /// \param other The path to copy
    ///
    void DeepCopy(
        const Path& other);

    ///
    /// \brief Parse a path in folder, filename and extension
    /// \param path The path to parse
    /// \param folder The folder of the file
    /// \param filename The file name
    /// \param ext The extension of the file
    ///
    static void parseFileName(
        const String& path,
        String& folder,
        String& filename,
        String& ext);

#ifndef SWIG
    ///
    /// \brief Test if file exist on the computer
    /// \return If file exist on the computer
    ///
    bool isFileExist() const;

    ///
    /// \brief Test if file exist on the computer
    /// \param path The path of the file
    /// \return If file exist on the computer
    ///
    static bool isFileExist(
        const Path& path);
#endif

    ///
    /// \brief Test if file exist on the computer
    /// \param path The path of the file
    /// \return If file exist on the computer
    ///
    static bool isFileExist(
        const String& path);

    ///
    /// \brief Test if file exist on the computer and is accessible
    /// \return If file is readable
    ///
    bool isFileReadable() const;

#ifndef SWIG
    ///
    /// \brief Test if folder exists on the computer
    /// \return If folder exists
    ///
    bool isFolderExist() const;

    ///
    /// \brief Test if folder exists on the computer
    /// \param path The path in which to test if the folder exists
    /// \return If folder exists
    ///
    static bool isFolderExist(
        const Path& path);
#endif

    ///
    /// \brief Test if folder exists on the computer
    /// \param path The path in which to test if the folder exists
    /// \return If folder exists
    ///
    static bool isFolderExist(
        const String& path);

    ///
    /// \brief To create folder on the computer
    ///
    void createFolder() const;

    ///
    /// /brief Return current directory
    /// \return Teh current directory
    ///
    static String currentDir();

    ///
    /// \brief Return relative path to current working directory
    /// \return The relative path to curent working directory
    ///
    String relativePath() const;

#ifndef SWIG
    ///
    /// \brief Return relative path to the specified folder
    /// \param relativeTo Relative to that path
    /// \return The relative path
    ///
    String relativePath(
        const String &relativeTo) const;

    ///
    /// \brief Return relative path of a specified path to the specified folder
    /// \param path The path to find the relative path from
    /// \param relativeTo Relative to that path
    /// \return The relative path
    ///
    static String relativePath(
        const Path &path,
        const String &relativeTo);

    ///
    /// \brief Return the absolute path relative to root
    /// \param path The path to find the path
    /// \return The absolute folder
    ///
    /// On Windows, the current implementation only works if the file is on the
    /// C drive.
    ///
    static String absoluteFolder(
        const Path &path);

    ///
    /// \brief Return the absolute folder relative to root
    /// \return The absolute folder relative to root
    ///
    String absoluteFolder() const;

#endif

    ///
    /// \brief Return the absolute path relative to root
    /// \return The absolute path relative to root
    ///
    String absolutePath() const;

    ///
    /// \brief Return the path to Unix format
    /// \param path The path to convert
    /// \return The path in Unix format
    ///
    static String toUnixFormat(
        const String& path);

    ///
    /// \brief Return the path to Windows format
    /// \param path The path to convert
    /// \return The path in Windows format
    ///
    static String toWindowsFormat(
        const String& path);

    ///
    /// \brief Return original path as it was at constructor time
    /// \return The original path
    ///
    const String& originalPath() const;

    ///
    /// \brief Return the folder of the file
    /// \return The folder of the file
    ///
    const String& folder() const;

    ///
    /// \brief Set the filename
    /// \param name The filename
    ///
    void setFilename(
        const String& name);

    ///
    /// \brief Return the filename
    /// \return The filename
    ///
    const String& filename() const;

    ///
    /// \brief Set the extension
    /// \param ext The extension
    ///
    void setExtension(
        const String& ext);

    ///
    /// \brief Return the extension of the file
    /// \return The extension of the file
    ///
    const String& extension() const;


protected:
    ///
    /// \brief Set if folder is absolute
    ///
    void setIsFolderAbsolute();

    std::shared_ptr<String>
    m_originalPath; ///< The original path at construction time
    std::shared_ptr<String> m_folder; ///< The folder
    std::shared_ptr<bool> m_isFolderAbsolute; ///< If folder is absolute
    std::shared_ptr<String> m_filename; ///< The filename
    std::shared_ptr<String> m_extension; ///< The extension
};

}
}
}

#endif // BIORBD_UTILS_PATH_H
