#define BIORBD_API_EXPORTS
#include "../include/s2mPath.h"

s2mPath::s2mPath()
    : s2mString("")
{

}

s2mPath::s2mPath(const char *c)
    : s2mString(processInputForS2MSTRINGcstr(c))
{
    parseFileName();
}

s2mPath::s2mPath(const s2mString &s)
    : s2mString(processInputForS2MSTRINGcstr(s))
{
    parseFileName();
}

s2mPath::s2mPath(const std::basic_string<char> &c)
    : s2mString(processInputForS2MSTRINGcstr(c))
{
    parseFileName();
}

s2mPath::~s2mPath()
{

}

bool s2mPath::isFileExist() const
{
    return isFileExist(static_cast<s2mString>(*this));
}
bool s2mPath::isFileExist(const s2mPath& path)
{
    return isFileExist(static_cast<s2mString>(path));
}
bool s2mPath::isFileExist(const s2mString& path)
{
    if (FILE *file = fopen(path.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

bool s2mPath::isFolderExist() const
{
    return isFolderExist(*this);
}

bool s2mPath::isFolderExist(const s2mPath &path)
{
	return isFolderExist(path.folder());
}

bool s2mPath::isFolderExist(const s2mString & path)
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
#elif _WIN32
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

void s2mPath::parseFileName(const s2mString &path, s2mString &folder, s2mString &filename, s2mString &extension)
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
/*void s2mPath::parseFileName(const s2mPath &path, s2mString &folder, s2mString &filename, s2mString &extension){
    return parseFileName(base::path, folder, filename, extension);
}*/

void s2mPath::parseFileName()
{
    parseFileName(*this, m_path, m_filename, m_extension);
}

s2mString s2mPath::processInputForS2MSTRINGcstr(const s2mString &path)
{
    s2mString folder, filename, extension;
    parseFileName(path, folder, filename, extension);
    return folder + filename + "." + extension;
}



void s2mPath::setRelativePath(const s2mString &path)
{
    m_path = getRelativePath(path);
}

s2mString s2mPath::getRelativePath()  const{
    s2mString currentDir = getCurrentDir();
    s2mString out(getRelativePath(currentDir));
    return out;
}

s2mString s2mPath::getRelativePath(const s2mString& path) const
{
    s2mString me(*this);
    s2mString currentDir(path);

    s2mString meFirstPart("");
    s2mString currentDirFirstPart("");

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

    s2mString outPath;
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

s2mString s2mPath::getAbsolutePath() const
{
#ifdef _WIN64
    s2mString base("C:\\");
#elif _WIN32
    s2mString base("C:\\");
#else
    s2mString base("/");
#endif
    s2mString test(base + getRelativePath(base) + m_filename + m_extension);
    return base + getRelativePath(base) + m_filename + m_extension;
}

const s2mString &s2mPath::folder() const
{
    return m_path;
}

const s2mString& s2mPath::filename() const
{
    return m_filename;
}

const s2mString& s2mPath::extension() const
{
    return m_extension;
}

const char * s2mPath::getCurrentDir()
{
    #ifdef _WIN64
        return _getcwd(nullptr, 0);
    #elif _WIN32
        return _getcwd(nullptr, 0);
    #else
        return getcwd(nullptr, 0);
#endif
}

void s2mPath::createFolder() const
{
    s2mString tp(folder());
    s2mString tp2(tp);

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
            if (!isFolderExist(static_cast<s2mString>(tp.substr(0, sepTrack)))){
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
