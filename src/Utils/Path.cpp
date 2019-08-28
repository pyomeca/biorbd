#define BIORBD_API_EXPORTS
#include "Utils/Path.h"

#include "Utils/String.h"

#if defined(_WIN32) || defined(_WIN64)
    #include <direct.h>
    #include <Windows.h>
    #undef max
#else
    #include <sys/stat.h>
    #include <unistd.h>
#endif

biorbd::utils::Path::Path() :
    m_path(std::make_shared<biorbd::utils::String>()),
    m_originalPath(std::make_shared<biorbd::utils::String>()),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{

}

biorbd::utils::Path::Path(const char *c) :
    m_path(std::make_shared<biorbd::utils::String>()),
    m_originalPath(std::make_shared<biorbd::utils::String>(c)),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{
    parseFileName(*m_originalPath, *m_folder, *m_filename, *m_extension);
    setIsFolderAbsolute();
    setPath();
}

biorbd::utils::Path::Path(const biorbd::utils::String &s) :
    m_path(std::make_shared<biorbd::utils::String>()),
    m_originalPath(std::make_shared<biorbd::utils::String>(s)),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{
    parseFileName(*m_originalPath, *m_folder, *m_filename, *m_extension);
    setIsFolderAbsolute();
    setPath();
}

biorbd::utils::Path::Path(const std::basic_string<char> &c) :
    m_path(std::make_shared<biorbd::utils::String>()),
    m_originalPath(std::make_shared<biorbd::utils::String>(c)),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{
    parseFileName(*m_originalPath, *m_folder, *m_filename, *m_extension);
    setIsFolderAbsolute();
    setPath();
}

biorbd::utils::Path biorbd::utils::Path::DeepCopy() const
{
    biorbd::utils::Path copy;
    *copy.m_path = *m_path;
    *copy.m_originalPath = *m_originalPath;
    *copy.m_folder = *m_folder;
    *copy.m_isFolderAbsolute = *m_isFolderAbsolute;
    *copy.m_filename = *m_filename;
    *copy.m_extension = *m_extension;
    return copy;
}

void biorbd::utils::Path::DeepCopy(const Path &other)
{
    m_path = std::make_shared<biorbd::utils::String>(*other.m_path);
    m_originalPath = std::make_shared<biorbd::utils::String>(*other.m_originalPath);
    m_folder = std::make_shared<biorbd::utils::String>(*other.m_folder);
    m_isFolderAbsolute = std::make_shared<bool>(*other.m_isFolderAbsolute);
    m_filename = std::make_shared<biorbd::utils::String>(*other.m_filename);
    m_extension = std::make_shared<biorbd::utils::String>(*other.m_extension);
}

bool biorbd::utils::Path::isFileExist() const
{
    return isFileExist(absolutePath());
}
bool biorbd::utils::Path::isFileExist(const Path& path)
{
    return isFileExist(path.absolutePath());
}
bool biorbd::utils::Path::isFileExist(const biorbd::utils::String& path)
{
    if (FILE *file = fopen(path.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

bool biorbd::utils::Path::isFolderExist() const
{
    return isFolderExist(*this);
}

bool biorbd::utils::Path::isFolderExist(const Path &path)
{
	return isFolderExist(path.folder());
}

bool biorbd::utils::Path::isFolderExist(const biorbd::utils::String & path)
{
#if defined(_WIN32) || defined(_WIN64)
	if (GetFileAttributesA(path.c_str()) == INVALID_FILE_ATTRIBUTES)
		return false; // Le path est invalide
	else
		// Si on est ici, c'est que quelque chose existe, s'assurer que ça ne soit pas un fichier
		if (isFileExist(path))
			return false;
		else
            return true;
#else
    struct stat statbuf;
    if (stat(path.c_str(), &statbuf) != -1) {
        return true;
    } else {
        return false;
    }
#endif

}

void biorbd::utils::Path::parseFileName(
        const biorbd::utils::String &path,
        biorbd::utils::String &folder,
        biorbd::utils::String &filename,
        biorbd::utils::String &extension)
{
    size_t sep(path.find_last_of("/")); // Assume UNIX formalism
    if (sep == std::string::npos)
        sep = path.find_last_of("\\"); // If nothing is found check for windows formalism

    // Stocker le folder
    if (sep == std::string::npos) // If no separator is found, then there is no separator and therefore the path is ./
        folder = "";
    else
        folder = path.substr(0,sep) + "/";
    if (folder.find("./") == 0) // Remove the leading ./ if necessary
        folder = folder.substr(2);

    // Stocker l'extension
    size_t ext(path.find_last_of("."));
    if (ext != SIZE_MAX)
        extension = path.substr(ext+1);
    else
        extension = "";

    // Stocker le nom de fichier
    filename = path.substr(sep+1, ext-sep-1);
}

biorbd::utils::String biorbd::utils::Path::relativePath()  const{
    return relativePath(*this, currentDir());
}

biorbd::utils::String biorbd::utils::Path::relativePath(const biorbd::utils::String& relativeTo) const
{
    return relativePath(*this, relativeTo);
}

biorbd::utils::String biorbd::utils::Path::relativePath(const biorbd::utils::Path &path, const biorbd::utils::String &relativeTo)
{
    biorbd::utils::String me(path.absolutePath());
    biorbd::utils::String currentDir(relativeTo);

    biorbd::utils::String meFirstPart("");
    biorbd::utils::String currentDirFirstPart("");

    // Set the separator to the 0 position
    size_t sepMe = std::string::npos;
    size_t sepCurrentDir = std::string::npos;
    do {
        // cut according to previous separator
        me = me.substr(sepMe+1);
        currentDir = currentDir.substr(sepCurrentDir+1);

        // Find the next separator
        sepMe = me.find_first_of("/"); // If UNIX formalism
        if (sepMe == std::string::npos)
            sepMe = me.find_first_of("\\"); // Otherwise assume window
        sepCurrentDir = currentDir.find_first_of("/");
        if (sepCurrentDir == std::string::npos)
            sepCurrentDir = currentDir.find_first_of("\\");

        // Separate first and last part
        meFirstPart = me.substr(0, sepMe);
        currentDirFirstPart = currentDir.substr(0, sepCurrentDir);

        // While the first part are equal, we still can advance to find de closest relative part
    } while(!meFirstPart.compare(currentDirFirstPart));

    biorbd::utils::String outPath;
    while(currentDir.compare("")) { // Tant que currentDir n'est pas vide, reculer
        // Trouver le prochain séparateur
        sepCurrentDir = currentDir.find_first_of("/");
        if (sepCurrentDir == std::string::npos)
            sepCurrentDir = currentDir.find_first_of("\\");

        // Séparer la première et la dernière partie
        if (sepCurrentDir != std::string::npos){ // -1 Si on est au dernier dossier et que celui-ci ne se termine pas par /
            currentDirFirstPart = currentDir.substr(0, sepCurrentDir);
            currentDir = currentDir.substr(sepCurrentDir+1);
        }
        else
            currentDir = "";

        outPath += "../";
        // Tant que les premières parties sont égales, continuer à avancer dans le path
    };
    if (!outPath.compare("") && me.find("../") != 0)
        outPath += "./";

    outPath += me;

    return outPath;
}

biorbd::utils::String biorbd::utils::Path::absoluteFolder(const biorbd::utils::Path &path)
{
#if defined(_WIN32) || defined(_WIN64)
    biorbd::utils::String base("C:\\");
#else
    biorbd::utils::String base("/");
#endif
    return base + relativePath(path, base);
}

biorbd::utils::String biorbd::utils::Path::absoluteFolder() const
{
    if (*m_isFolderAbsolute)
        return *m_folder;
    else
        return currentDir() + *m_folder;
}

biorbd::utils::String biorbd::utils::Path::absolutePath() const
{
    if (m_filename->compare("")){
        if (m_extension->compare(""))
            return absoluteFolder() + *m_filename + "." + *m_extension;
        else
            return absoluteFolder() + *m_filename;
    } else
        return absoluteFolder();
}

const biorbd::utils::String &biorbd::utils::Path::originalPath() const
{
    return *m_originalPath;
}

const biorbd::utils::String &biorbd::utils::Path::folder() const
{
    return *m_folder;
}

void biorbd::utils::Path::setFilename(const biorbd::utils::String& name)
{
    *m_filename = name;
}

const biorbd::utils::String& biorbd::utils::Path::filename() const
{
    return *m_filename;
}

void biorbd::utils::Path::setExtension(const biorbd::utils::String &ext)
{
    *m_extension = ext;
}

void biorbd::utils::Path::setPath()
{
    biorbd::utils::String path;

    if (m_filename->compare("")){
        if (m_extension->compare(""))
            *m_path = absolutePath() + *m_filename + "." + *m_extension;
        else
            *m_path = absolutePath() + *m_filename;
    } else
        *m_path = path;
}

void biorbd::utils::Path::setIsFolderAbsolute()
{
#if defined(_WIN32) || defined(_WIN64)
    biorbd::utils::String base("C:\\");
#else
    biorbd::utils::String base("/");
#endif
    size_t pos(m_folder->find_first_of(base.c_str()));
    if (pos == 0)
        *m_isFolderAbsolute = true;
    else
        *m_isFolderAbsolute = false;
}

const biorbd::utils::String& biorbd::utils::Path::extension() const
{
    return *m_extension;
}

biorbd::utils::String biorbd::utils::Path::currentDir()
{
    char buff[FILENAME_MAX];
#if defined(_WIN32) || defined(_WIN64)
     _getcwd(buff, FILENAME_MAX);
#else
    getcwd(buff, FILENAME_MAX);
#endif
    return biorbd::utils::String(buff) + "/";
}

void biorbd::utils::Path::createFolder() const
{
    biorbd::utils::String tp(folder());
    biorbd::utils::String tp2(tp);

    size_t sep = std::string::npos;
    size_t sepTrack = 0;
    do {
        // Découper en fonction de la position précédente du séparateur
        tp2 = tp2.substr(sep+1);

        // Trouver le prochain séparateur
        sep = tp2.find_first_of("/"); // Try with UNIX formalism
        if (sep == std::string::npos)
             sep = tp2.find_first_of("\\"); // If nothing is found try with windows formalism

        if (sep != std::string::npos){
            sepTrack += sep + 1 ;

            // Séparer la première et la dernière partie
            if (!isFolderExist(static_cast<biorbd::utils::String>(tp.substr(0, sepTrack)))){
#if defined(_WIN32) || defined(_WIN64)
    _mkdir(tp.substr(0, sepTrack).c_str());
#else
    mkdir(tp.substr(0, sepTrack).c_str(), 0777);
#endif
            }
        }
    } while (sep != std::string::npos);
}
