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

class BIORBD_API Path
{
public: 
    Path();
    Path(const char *c);
    Path(const biorbd::utils::String &s);
    Path(const std::basic_string<char> &c);
    biorbd::utils::Path DeepCopy();

    // parser un path en folder, filename,e extension
    static void parseFileName(
            const biorbd::utils::String& path,
            biorbd::utils::String& folder,
            biorbd::utils::String& filename,
            biorbd::utils::String& ext);

    // Tests sur les fichiers
    bool isFileExist() const;
    static bool isFileExist(const biorbd::utils::Path&);
    static bool isFileExist(const biorbd::utils::String&);
    bool isFolderExist() const;
    static bool isFolderExist(const biorbd::utils::Path&);
    static bool isFolderExist(const biorbd::utils::String&);
    void createFolder() const;

    static biorbd::utils::String currentDir();
    biorbd::utils::String relativePath() const; // Relative to current working directory
    biorbd::utils::String relativePath(const biorbd::utils::String &relativeTo) const; // Relative to that path
    static biorbd::utils::String relativePath(const biorbd::utils::Path &path, const biorbd::utils::String &relativeTo); // Relative to that path
    static biorbd::utils::String absoluteFolder(const biorbd::utils::Path &path); // Relative to root
    biorbd::utils::String absoluteFolder() const; // Relative to root
    biorbd::utils::String absolutePath() const; // Relative to root

    // Accessor
    const biorbd::utils::String& originalPath() const;
    const biorbd::utils::String& folder() const;
    const biorbd::utils::String& filename() const;
    void setFilename(const biorbd::utils::String& name);
    const biorbd::utils::String &extension() const;
    void setExtension(const biorbd::utils::String& ext);

protected:
    void setPath();
    void setIsFolderAbsolute();

    std::shared_ptr<biorbd::utils::String> m_path;
    std::shared_ptr<biorbd::utils::String> m_originalPath;
    std::shared_ptr<biorbd::utils::String> m_folder;
    std::shared_ptr<bool> m_isFolderAbsolute;
    std::shared_ptr<biorbd::utils::String> m_filename;
    std::shared_ptr<biorbd::utils::String> m_extension;
};

}}

#endif // BIORBD_UTILS_PATH_H
