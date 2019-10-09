#define BIORBD_API_EXPORTS
#include "Utils/IfStream.h"

#include <boost/lexical_cast.hpp>
#include <fstream>
#include "Utils/Error.h"
#include "Utils/Equation.h"

// Constructeur
biorbd::utils::IfStream::IfStream() :
    m_isOpen(std::make_shared<bool>(false)),
    m_ifs(std::make_shared<std::ifstream>()),
    m_path(std::make_shared<biorbd::utils::Path>())
{

}
biorbd::utils::IfStream::IfStream(
        const biorbd::utils::Path& path,
        std::ios_base::openmode mode = std::ios_base::in ) :
    m_isOpen(std::make_shared<bool>(false)),
    m_ifs(std::make_shared<std::ifstream>()),
    m_path(std::make_shared<biorbd::utils::Path>(path))
{
    open(m_path->absolutePath().c_str(), mode);
}
biorbd::utils::IfStream::IfStream(
        const char* path,
        std::ios_base::openmode mode = std::ios_base::in ) :
    m_isOpen(std::make_shared<bool>(false)),
    m_ifs(std::make_shared<std::ifstream>()),
    m_path(std::make_shared<biorbd::utils::Path>(path))
{
    open(m_path->absolutePath().c_str(), mode);
}


// Ouvrir le fichier
bool biorbd::utils::IfStream::open(
        const biorbd::utils::Path& path,
        std::ios_base::openmode mode = std::ios_base::in )
{
    biorbd::utils::Error::check(path.isFileExist(), path.absolutePath() + " could not be loaded");
    *m_ifs = std::ifstream(path.relativePath().c_str(), mode);
    *m_isOpen = true;
    return *m_isOpen;
}

// Lire le fichier
bool biorbd::utils::IfStream::readSpecificTag(
        const biorbd::utils::String& tag,
        biorbd::utils::String& text){
    reachSpecificTag(tag);
    read(text);
    return true;
}
bool biorbd::utils::IfStream::reachSpecificTag(const biorbd::utils::String& tag){
    biorbd::utils::String text;
    while (read(text))
        if (!text.tolower().compare(tag))
            return true;

    biorbd::utils::String outMessage(tag + " parameter could not be found in Data file..");
    biorbd::utils::Error::raise(outMessage);
    return false; // Il est impossible qu'on se rende ici, mais c'est mieux d'avoir un return pour le compilateur
}

int biorbd::utils::IfStream::countTagsInAConsecutiveLines(const biorbd::utils::String &tag)
{
    // Se souvenir où on était dans le fichier
    long positionInFile(m_ifs->tellg());
    biorbd::utils::String text;
    int nMarkers(0);

    // Lire le premier mot de la ligne
    while (read(text))
        // S'il est comme le tag, saute à la prochaine ligne et on recommence
        if (!text.compare(tag)){
            getline(text);
            ++nMarkers;
        }
        else
            break;

    // Remettre le fichier au point d'origine
    m_ifs->seekg(positionInFile);
    return nMarkers;
}

bool biorbd::utils::IfStream::read(biorbd::utils::String& text){
    bool out(*m_ifs >> text);

    if (out == 0)
        return out;

    if (!text(0,1).compare("//")){ // si c'est un commentaire par //
        getline(text);
        read(text);
    }
    else if (!text(0,1).compare("/*")){ // si c'est une commentaire par / *
        while (readIgnoreCommentedLine(text)){
            if (!text(0,1).compare("*/") || (text.length()>=2 && !text(static_cast<unsigned int>(text.length()-2),static_cast<unsigned int>(text.length()-1)).compare("*/")))
                break;
        }
        read(text);
    }
    return out;
}
bool biorbd::utils::IfStream::readIgnoreCommentedLine(biorbd::utils::String& text){
    bool out(*m_ifs >> text);
    return out;
}
bool biorbd::utils::IfStream::read(double& d){
    std::map<biorbd::utils::Equation, double> dumb;
    return read(d, dumb);
}
bool biorbd::utils::IfStream::read(double& d, const std::map<biorbd::utils::Equation, double> &variables){
    biorbd::utils::Equation tp;
    bool out(read(tp));
    // gérer le cas d'une équation
    try {
        d = biorbd::utils::Equation::resolveEquation(tp, variables);
    } catch (boost::bad_lexical_cast) {
        biorbd::utils::Error::raise("The following expression cannot be parsed properly: \"" + tp + "\"");
    }
    return out;
}
bool biorbd::utils::IfStream::read(int& i){
    biorbd::utils::String tp;
    bool out(read(tp));
    i = boost::lexical_cast<int>(tp);
    return out;
}
bool biorbd::utils::IfStream::read(unsigned int& i){
    biorbd::utils::String tp;
    bool out(read(tp));
    i = boost::lexical_cast<unsigned int>(tp);
    return out;
}
bool biorbd::utils::IfStream::read(bool& i){
    biorbd::utils::String tp;
    bool out(read(tp));
    i = boost::lexical_cast<bool>(tp);
    return out;
}
// Lire toute une ligne
void biorbd::utils::IfStream::getline(biorbd::utils::String& text){
    std::getline(*m_ifs, text);
}


// Fermer le fichier
bool biorbd::utils::IfStream::close(){
    m_ifs->close();
    return 1;
}

bool biorbd::utils::IfStream::eof()
{
    return m_ifs->eof();
}
