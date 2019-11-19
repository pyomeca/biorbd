#define BIORBD_API_EXPORTS
#include "Utils/IfStream.h"

#include <boost/lexical_cast.hpp>
#include <fstream>
#include "Utils/Error.h"
#include "Utils/Equation.h"

// Constructor
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


// Open a file
bool biorbd::utils::IfStream::open(
        const biorbd::utils::Path& path,
        std::ios_base::openmode mode = std::ios_base::in )
{
    biorbd::utils::Error::check(path.isFileExist(), path.absolutePath() + " could not be loaded");
    *m_ifs = std::ifstream(path.relativePath().c_str(), mode);
    *m_isOpen = true;
    return *m_isOpen;
}

// Read a file
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
    return false; // It is impossible to get here, but it's better to have a return for the compiler
}

int biorbd::utils::IfStream::countTagsInAConsecutiveLines(const biorbd::utils::String &tag)
{
    // Remember where we were in the file
    std::streamoff positionInFile(m_ifs->tellg());
    biorbd::utils::String text;
    int nMarkers(0);

    // Read the first word of the line
    while (read(text))
        // If same as tag, skip to the other line and restart
        if (!text.compare(tag)){
            getline(text);
            ++nMarkers;
        }
        else
            break;

    // Reset in file to the origine point
    m_ifs->seekg(positionInFile);
    return nMarkers;
}

bool biorbd::utils::IfStream::read(biorbd::utils::String& text){
    bool out(*m_ifs >> text);

    if (out == 0)
        return out;

    if (!text(0,1).compare("//")){ // If it's a comment by //
        getline(text);
        read(text);
    }
    else if (!text(0,1).compare("/*")){ // If it's a comment by / *
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
    // Manage in case of an equation
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
// Read the entire line
void biorbd::utils::IfStream::getline(biorbd::utils::String& text){
    std::getline(*m_ifs, text);
}


// Close the file
bool biorbd::utils::IfStream::close(){
    m_ifs->close();
    return 1;
}

bool biorbd::utils::IfStream::eof()
{
    return m_ifs->eof();
}
