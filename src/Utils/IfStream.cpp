#define BIORBD_API_EXPORTS
#include "Utils/IfStream.h"

#include <boost/lexical_cast.hpp>
#include <fstream>
#include "Utils/Error.h"
#include "Utils/Equation.h"

// Constructeur
s2mIfStream::s2mIfStream(const s2mPath& path, std::ios_base::openmode mode = std::ios_base::in ) :
    m_path(path)
{
    open(path.c_str(), mode);
}
s2mIfStream::s2mIfStream(const char* path, std::ios_base::openmode mode = std::ios_base::in ) :
    m_path(path)
{
    open(path, mode);
}
s2mIfStream::s2mIfStream( ){
    m_isOpen = false;
}

s2mIfStream::~s2mIfStream(){
    delete m_ifs;
}


// Ouvrir le fichier
bool s2mIfStream::open(const s2mPath& path, std::ios_base::openmode mode = std::ios_base::in ) {
    return open(path.c_str(), mode);
}
bool s2mIfStream::open(const char* path, std::ios_base::openmode mode = std::ios_base::in ) {
    m_ifs = new std::ifstream(path, mode);
    s2mError::s2mAssert(m_ifs!=nullptr, path + s2mString(" file could not be opened"));
    m_isOpen = true;
    return m_isOpen;
}


// Lire le fichier
bool s2mIfStream::readSpecificTag(const s2mString& tag, s2mString& text){
    reachSpecificTag(tag);
    read(text);
    return true;
}
bool s2mIfStream::reachSpecificTag(const s2mString& tag){
    s2mString text;
    while (read(text))
        if (!text.tolower().compare(tag))
            return true;

    s2mString outMessage = tag + " parameter could not be found in Data file..";
    s2mError::s2mAssert(0, outMessage);
    return false; // Il est impossible qu'on se rende ici, mais c'est mieux d'avoir un return pour le compilateur
}

int s2mIfStream::countTagsInAConsecutiveLines(const s2mString &tag)
{
    // Se souvenir où on était dans le fichier
    long positionInFile(m_ifs->tellg());
    s2mString text;
    int nTags(0);

    // Lire le premier mot de la ligne
    while (read(text))
        // S'il est comme le tag, saute à la prochaine ligne et on recommence
        if (!text.compare(tag)){
            getline(text);
            ++nTags;
        }
        else
            break;

    // Remettre le fichier au point d'origine
    m_ifs->seekg(positionInFile);
    return nTags;
}

bool s2mIfStream::read(s2mString& text){
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
bool s2mIfStream::readIgnoreCommentedLine(s2mString& text){
    bool out(*m_ifs >> text);
    return out;
}
bool s2mIfStream::read(double& d){
    std::map<s2mEquation, double> dumb;
    return read(d, dumb);
}
bool s2mIfStream::read(double& d, const std::map<s2mEquation, double> &variables){
    s2mEquation tp;
    bool out(read(tp));
    // gérer le cas d'une équation
    d = s2mEquation::resolveEquation(tp, variables);
    return out;
}
bool s2mIfStream::read(int& i){
    s2mString tp;
    bool out(read(tp));
    i = boost::lexical_cast<int>(tp);
    return out;
}
bool s2mIfStream::read(unsigned int& i){
    s2mString tp;
    bool out(read(tp));
    i = boost::lexical_cast<unsigned int>(tp);
    return out;
}
bool s2mIfStream::read(bool& i){
    s2mString tp;
    bool out(read(tp));
    i = boost::lexical_cast<bool>(tp);
    return out;
}
// Lire toute une ligne
void s2mIfStream::getline(s2mString& text){
    std::getline(*m_ifs, text);
}


// Fermer le fichier
bool s2mIfStream::close(){
    m_ifs->close();
    return 1;
}

bool s2mIfStream::eof()
{
    return m_ifs->eof();
}









