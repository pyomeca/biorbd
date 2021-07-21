#define BIORBD_API_EXPORTS
#include "Utils/IfStream.h"

#include <clocale>
#include <fstream>
#include "Utils/Error.h"
#include "Utils/Equation.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

// Constructor
utils::IfStream::IfStream() :
    m_isOpen(std::make_shared<bool>(false)),
    m_ifs(std::make_shared<std::ifstream>()),
    m_path(std::make_shared<utils::Path>())
{
    setlocale(LC_ALL, "C");
}
utils::IfStream::IfStream(
    const utils::Path& path,
    std::ios_base::openmode mode = std::ios_base::in ) :
    m_isOpen(std::make_shared<bool>(false)),
    m_ifs(std::make_shared<std::ifstream>()),
    m_path(std::make_shared<utils::Path>(path))
{
    open(m_path->absolutePath().c_str(), mode);
    setlocale(LC_ALL, "C");
}
utils::IfStream::IfStream(
    const char* path,
    std::ios_base::openmode mode = std::ios_base::in ) :
    m_isOpen(std::make_shared<bool>(false)),
    m_ifs(std::make_shared<std::ifstream>()),
    m_path(std::make_shared<utils::Path>(path))
{
    open(m_path->absolutePath().c_str(), mode);
    setlocale(LC_ALL, "C");
}


// Open a file
bool utils::IfStream::open(
    const utils::Path& path,
    std::ios_base::openmode mode = std::ios_base::in )
{
    utils::Error::check(path.isFileExist(),
                                path.absolutePath() + " could not be loaded");
    *m_ifs = std::ifstream(path.absolutePath().c_str(), mode);
    *m_isOpen = true;
    return *m_isOpen;
}

// Read a file
bool utils::IfStream::readSpecificTag(
    const utils::String& tag,
    utils::String& text)
{
    reachSpecificTag(tag);
    read(text);
    return true;
}
bool utils::IfStream::reachSpecificTag(const utils::String& tag)
{
    utils::String text;
    while (read(text))
        if (!text.tolower().compare(tag)) {
            return true;
        }

    utils::String outMessage(tag +
                                     " parameter could not be found in Data file..");
    utils::Error::raise(outMessage);
#ifdef _WIN32
    // It is impossible to get here, but it's better to have a return for the compiler
    return false;
#endif
}

int utils::IfStream::countTagsInAConsecutiveLines(
    const utils::String &tag)
{
    // Remember where we were in the file
    std::streamoff positionInFile(m_ifs->tellg());
    utils::String text;
    int nMarkers(0);

    // Read the first word of the line
    while (read(text))
        // If same as tag, skip to the other line and restart
        if (!text.compare(tag)) {
            getline(text);
            ++nMarkers;
        } else {
            break;
        }

    // Reset in file to the origine point
    m_ifs->seekg(positionInFile);
    return nMarkers;
}

bool utils::IfStream::read(utils::String& text)
{
    bool out(*m_ifs >> text);

    if (out == 0) {
        return out;
    }

    if (!text(0,1).compare("//")) { // If it's a comment by //
        getline(text);
        read(text);
    } else if (!text(0,1).compare("/*")) { // If it's a comment by / *
        while (readAWord(text)) {
            if (!text(0,1).compare("*/") || (text.length()>=2
                                             && !text(static_cast<unsigned int>(text.length()-2),
                                                     static_cast<unsigned int>(text.length()-1)).compare("*/"))) {
                break;
            }
        }
        read(text);
    }
    return out;
}
bool utils::IfStream::readAWord(utils::String& text)
{
    bool out(*m_ifs >> text);
    return out;
}
bool utils::IfStream::read(
    double& val)
{
    std::map<utils::Equation, double> dumb;
    return read(val, dumb);
}
#ifdef BIORBD_USE_CASADI_MATH
bool utils::IfStream::read(
    RBDLCasadiMath::MX_Xd_SubMatrix val)
{
    std::map<utils::Equation, double> dumb;
    return read(val, dumb);
}
#endif
bool utils::IfStream::read(
    double& result,
    const std::map<utils::Equation, double> &variables)
{
    utils::Equation tp;
    bool out(read(tp));
    // Manage in case of an equation
    try {
        result = utils::Equation::evaluateEquation(tp, variables);
    } catch (std::runtime_error) {
        utils::Error::raise("The following expression cannot be parsed properly: \""
                                    + tp + "\"");
    }
    return out;
}
#ifdef BIORBD_USE_CASADI_MATH
bool utils::IfStream::read(
    RBDLCasadiMath::MX_Xd_SubMatrix result,
    const std::map<utils::Equation, double> &variables)
{
    utils::Equation tp;
    bool out(read(tp));
    // Manage in case of an equation
    try {
        result = utils::Equation::evaluateEquation(tp, variables);
    } catch (std::runtime_error) {
        utils::Error::raise("The following expression cannot be parsed properly: \""
                                    + tp + "\"");
    }
    return out;
}
#endif
bool utils::IfStream::read(
    int& val)
{
    utils::String tp;
    bool out(read(tp));
    val = std::stoi(tp);
    return out;
}
bool utils::IfStream::read(
    unsigned int& val)
{
    utils::String tp;
    bool out(read(tp));
    val = static_cast<unsigned int>(std::stoul(tp));
    return out;
}
bool utils::IfStream::read(
    bool& val)
{
    utils::String tp;
    bool out(read(tp));
    val = std::stoi(tp) != 0;
    return out;
}
// Read the entire line
void utils::IfStream::getline(utils::String& text)
{
    std::getline(*m_ifs, text);
}


// Close the file
bool utils::IfStream::close()
{
    m_ifs->close();
    return 1;
}

bool utils::IfStream::eof()
{
    return m_ifs->eof();
}
