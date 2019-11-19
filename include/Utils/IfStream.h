#ifndef BIORBD_UTILS_IFSTREAM_H
#define BIORBD_UTILS_IFSTREAM_H

#include <map>
#include "biorbdConfig.h"
#include "Utils/Path.h"
#include <iostream>

namespace biorbd {
namespace utils {
class Equation;
///
/// \brief Class IfStream
///
class BIORBD_API IfStream
{
public:
    /// 
    /// \brief Construct IfStream
    ///
    IfStream();
    /// 
    /// \brief Construct IfStream
    /// \param path The stream path
    /// \param mode The stream mode
    ///
    IfStream(
            const biorbd::utils::Path& path,
            std::ios_base::openmode mode );
    /// 
    /// \brief Construct IfStream
    /// \param path The stream path
    /// \param mode The stream mode
    ///
    IfStream(
            const char* path,
            std::ios_base::openmode mode );
    /// 
    /// \brief Open the file
    /// \param path The file path
    /// \param mode The open mode
    /// \return True (opened) or false
    ///
    bool open(
            const biorbd::utils::Path& path,
            std::ios_base::openmode mode );

    /// 
    /// \brief Read text
    /// \param The text to read
    /// \return True (text read) or False
    ///
    bool read(biorbd::utils::String& text);

    /// 
    /// \brief Read text without the commented lines
    /// \param The text to read
    /// \return True (text read) or False
    ///
    bool readIgnoreCommentedLine(biorbd::utils::String&text);

    ///
    /// \brief Read text and gives it a number TODO:
    /// \param i The number to give
    /// \return True (text read) or False
    ///
    bool read(int&i);

    ///
    /// \brief Read text and gives it a number TODO:
    /// \param bool True or False
    /// \return True (text read) or False
    ///
    bool read(bool&bool);
    ///
    /// \brief Read text and gives it a number TODO:
    /// \param i The number to give
    /// \return True (text read) or False
    ///
    bool read(unsigned int&i);
    ///
    /// \brief Read text and gives it a number TODO:
    /// \param d The number to give
    /// \return True (text read) or False
    ///
    bool read(double&d);

    ///
    /// \brief Read equation
    /// \param d The number to give
    /// \param variables The variables
    /// \return True (equation read) or False
    ///
    bool read(
            double&d,
            const std::map<biorbd::utils::Equation, double> &variables);
    ///
    /// \brief Read text at a specific tag
    /// \param tag Tag
    /// \param text The text 
    /// \return True (text read) or False
    ///
    bool readSpecificTag(
            const biorbd::utils::String&tag,
            biorbd::utils::String&text);
    ///
    /// \brief Scroll a file until tag is reached and return the number of skipped lines
    /// \param tag Tag
    /// \return True (text read) or False
    ///
    bool reachSpecificTag(const biorbd::utils::String&tag); 

    ///
    /// \brief Counts the number of lines starting with the same tag and then brings it back to the beginning
    /// \param tag Tag
    /// \return The number of lines starting with the same tag
    ///
    int countTagsInAConsecutiveLines(const biorbd::utils::String&tag); 

    ///
    /// \brief Read a whole line
    /// \param text The text to read
    ///
    void getline(biorbd::utils::String&text);


    ///
    /// \brief Close the file
    ///
    bool close();

    ///
    /// \brief Return if end of file
    ///
    bool eof();

protected:
    std::shared_ptr<bool> m_isOpen;///< If file is open

private:
    std::shared_ptr<std::ifstream> m_ifs;///< If stream
    std::shared_ptr<biorbd::utils::Path> m_path;///<The path of the stream
};

}}

#endif // BIORBD_UTILS_IFSTREAM_H

