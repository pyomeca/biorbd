#define BIORBD_API_EXPORTS
#include "Utils/Path.h"

#ifdef _WIN64
    #include <direct.h>
    #include <Windows.h>
    #undef max
#elseif _WIN32
    #include <direct.h>
    #include <Windows.h>
    #undef max
#else
    #include <sys/stat.h>
    #include <unistd.h>
#endif

biorbd::utils::Path::Path()
    : biorbd::utils::String("")
{

}

biorbd::utils::Path::Path(const char *c)
    : biorbd::utils::String(processInputForStringCstr(c))
{
    parseFileName();
}

biorbd::utils::Path::Path(const biorbd::utils::String &s)
    : biorbd::utils::String(processInputForStringCstr(s))
{
    parseFileName();
}

biorbd::utils::Path::Path(const std::basic_string<char> &c)
    : biorbd::utils::String(processInputForStringCstr(c))
{
    parseFileName();
}

biorbd::utils::Path::~Path()
{

}

bool biorbd::utils::Path::isFileExist() const
{
    return isFileExist(static_cast<biorbd::utils::String>(*this));
}
bool biorbd::utils::Path::isFileExist(const Path& path)
{
    return isFileExist(static_cast<biorbd::utils::String>(path));
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
#ifdef _WIN64
	if (GetFileAttributesA(path.c_str()) == INVALID_FILE_ATTRIBUTES)
		return false; // Le path est invalide
	else
		// Si on est ici, c'est que quelque chose existe, s'assurer que ça ne soit pas un fichier
		if (isFileExist(path))
			return false;
		else
			return true;
#elseif _WIN32
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
    size_t sep(path.find_last_of("\\")); //si on utilise le formalisme de windows
    if (sep == std::string::npos)
        sep = path.find_last_of("/"); // Si on a rien trouvé, on est en formalisme linux

    // Stocker le folder
    if (sep == std::string::npos) // Si on a toujours rien, c'est que qu'on est déjà dans le bon dossier
        folder = "./";
    else
        folder = path.substr(0,sep) + "/";

    // Stocker l'extension
    size_t ext(path.find_last_of("."));
    extension = path.substr(ext+1);

    // Stocker le nom de fichier
    filename = path.substr(sep+1, ext-sep-1);
}
//void biorbd::utils::Path::parseFileName(const Path &path, biorbd::utils::String &folder, biorbd::utils::String &filename, biorbd::utils::String &extension){
//    return parseFileName(base::path, folder, filename, extension);
//}

void biorbd::utils::Path::parseFileName()
{
    parseFileName(*this, m_path, m_filename, m_extension);
}

biorbd::utils::String biorbd::utils::Path::processInputForStringCstr(const biorbd::utils::String &path)
{
    biorbd::utils::String folder, filename, extension;
    parseFileName(path, folder, filename, extension);
    return folder + filename + "." + extension;
}



void biorbd::utils::Path::setRelativePath(const biorbd::utils::String &path)
{
    m_path = getRelativePath(path);
}

biorbd::utils::String biorbd::utils::Path::getRelativePath()  const{
    biorbd::utils::String currentDir = getCurrentDir();
    biorbd::utils::String out(getRelativePath(currentDir));
    return out;
}

biorbd::utils::String biorbd::utils::Path::getRelativePath(const biorbd::utils::String& path) const
{
    biorbd::utils::String me(*this);
    biorbd::utils::String currentDir(path);

    biorbd::utils::String meFirstPart("");
    biorbd::utils::String currentDirFirstPart("");

    size_t sepMe = std::string::npos;
    size_t sepCurrentDir = std::string::npos;
    do {
        // Découper en fonction de la position précédente du séparateur
        me = me.substr(sepMe+1);
        currentDir = currentDir.substr(sepCurrentDir+1);

        // Trouver le prochain séparateur
        sepMe = me.find_first_of("\\"); //si on utilise le formalisme de windows
        if (sepMe == std::string::npos)
            sepMe = me.find_first_of("/"); // Si on a rien trouvé, on est en formalisme linux
        sepCurrentDir = currentDir.find_first_of("\\"); //si on utilise le formalisme de windows
        if (sepCurrentDir == std::string::npos)
            sepCurrentDir = currentDir.find_first_of("/"); // Si on a rien trouvé, on est en formalisme linux

        // Séparer la première et la dernière partie
        meFirstPart = me.substr(0, sepMe);
        currentDirFirstPart = currentDir.substr(0, sepCurrentDir);


        // Tant que les premières parties sont égales, continuer à avancer dans le path
    } while(!meFirstPart.compare(currentDirFirstPart));

    biorbd::utils::String outPath;
    while(currentDir.compare("")) { // Tant que currentDir n'est pas vide, reculer
        // Trouver le prochain séparateur
        sepCurrentDir = currentDir.find_first_of("\\"); //si on utilise le formalisme de windows
        if (sepCurrentDir == std::string::npos)
            sepCurrentDir = currentDir.find_first_of("/"); // Si on a rien trouvé, on est en formalisme linux

        // Séparer la première et la dernière partie
        if (sepCurrentDir != std::string::npos){ // -1 Si on est au dernier dossier et que celui-ci ne se termine pas par /
            currentDirFirstPart = currentDir.substr(0, sepCurrentDir);
            currentDir = currentDir.substr(sepCurrentDir+1);
        }
        else
            currentDir = "";

        outPath += "../";
        // Tant que les premières parties sont égales, continuer à avancer dans le path
    } ;

    outPath += me;

    return outPath;
}

biorbd::utils::String biorbd::utils::Path::getAbsolutePath() const
{
#ifdef _WIN64
    biorbd::utils::String base("C:\\");
#elseif _WIN32
    biorbd::utils::String base("C:\\");
#else
    biorbd::utils::String base("/");
#endif
    biorbd::utils::String test(base + getRelativePath(base) + m_filename + m_extension);
    return base + getRelativePath(base) + m_filename + m_extension;
}

const biorbd::utils::String &biorbd::utils::Path::folder() const
{
    return m_path;
}

const biorbd::utils::String& biorbd::utils::Path::filename() const
{
    return m_filename;
}

const biorbd::utils::String& biorbd::utils::Path::extension() const
{
    return m_extension;
}

const char * biorbd::utils::Path::getCurrentDir()
{
    #ifdef _WIN64
        return _getcwd(nullptr, 0);
    #elseif _WIN32
        return _getcwd(nullptr, 0);
    #else
        return getcwd(nullptr, 0);
#endif
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
        sep = tp2.find_first_of("\\"); //si on utilise le formalisme de windows
        if (sep == std::string::npos)
             sep = tp2.find_first_of("/"); // Si on a rien trouvé, on est en formalisme linux

        if (sep != std::string::npos){
            sepTrack += sep + 1 ;

            // Séparer la première et la dernière partie
            if (!isFolderExist(static_cast<biorbd::utils::String>(tp.substr(0, sepTrack)))){
#ifdef _WIN64
				_mkdir(tp.substr(0, sepTrack).c_str());
#elseif _WIN32
				_mkdir(tp.substr(0, sepTrack).c_str());
#else
                mkdir(tp.substr(0, sepTrack).c_str(), 0777);
#endif
            }
        }
    } while (sep != std::string::npos);
}
