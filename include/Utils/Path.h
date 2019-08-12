#ifndef BIORBD_UTILS_PATH_H
#define BIORBD_UTILS_PATH_H

#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd { namespace utils {

class BIORBD_API Path : public biorbd::utils::String
{
public: 
    Path();
    Path(const char *c);
    Path(const biorbd::utils::String &s);
    Path(const std::basic_string<char> &c);
    virtual ~Path();

    // parser un path en folder, filename,e extension
    static void parseFileName(
            const biorbd::utils::String& path,
            biorbd::utils::String& folder,
            biorbd::utils::String& filename,
            biorbd::utils::String& ext);
    void parseFileName();

    // Tests sur les fichiers
    bool isFileExist() const;
    static bool isFileExist(const biorbd::utils::Path&);
    static bool isFileExist(const biorbd::utils::String&);
    bool isFolderExist() const;
    static bool isFolderExist(const biorbd::utils::Path&);
    static bool isFolderExist(const biorbd::utils::String&);
    void createFolder() const;

    // Ces fonctions sont partielles et ne fonctionnent que dans le cas où il y a un certain tronc commun avec m_path
    void setRelativePath(const biorbd::utils::String&); // setter le path relatif à un dossier précis
    biorbd::utils::String getRelativePath() const; // Relative path par rapport au path courant
    biorbd::utils::String getRelativePath(const biorbd::utils::String& relPath) const; // relative path par rapport au chemin envoyé
    biorbd::utils::String getAbsolutePath() const;

    // Accessor
    const biorbd::utils::String& folder() const;
    const biorbd::utils::String& filename() const;
    const biorbd::utils::String &extension() const;
    static const char* getCurrentDir();

protected:
    // Parse appelé durant la liste d'initiation pour s2mString
    static biorbd::utils::String processInputForStringCstr(const biorbd::utils::String&);

    biorbd::utils::String m_path; // Comme il a été envoyé
    biorbd::utils::String m_filename;
    biorbd::utils::String m_extension;
};

}}

#endif // BIORBD_UTILS_PATH_H
