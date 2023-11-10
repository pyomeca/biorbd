#ifndef BIORBD_UTILS_IFSTREAM_H
#define BIORBD_UTILS_IFSTREAM_H

#include <map>
#include "biorbdConfig.h"
#include "Utils/Path.h"
#include <iostream>
#include "rbdl/rbdl_math.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class Equation;
class Vector3d;

///
/// \brief Wrapper for the an std::ifstream with increased capacities
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
    /// \param path The file path to open
    /// \param mode The open mode of "std::ios_base" base
    ///
    IfStream(
        const Path& path,
        std::ios_base::openmode mode );

    ///
    /// \brief Construct IfStream
    /// \param path The file path to open
    /// \param mode The open mode of "std::ios_base" base
    ///
    IfStream(
        const char* path,
        std::ios_base::openmode mode );

    ///
    /// \brief Open the file
    /// \param path The file path to open
    /// \param mode The open mode of "std::ios_base" base
    /// \return True on success
    ///
    bool open(
        const Path& path,
        std::ios_base::openmode mode );

    ///
    /// \brief Read a word in the file skipping the word if it is c-like commented
    /// \param text The text read (output)
    /// \return True on success
    ///
    bool read(
        String& text);

    ///
    /// \brief Read a word in the file
    /// \param text The text read (output)
    /// \return True on success
    ///
    bool readAWord(
        String& text);

    ///
    /// \brief Read an integer in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        int& val);

    ///
    /// \brief Read an boolean in the file
    /// \param val The value read (output)
    /// \return True on success
    ///
    bool read(
        bool& val);

    ///
    /// \brief Read an unsigned integer in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        size_t& val);

    ///
    /// \brief Read an double in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        double& val);
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Read an double in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        RBDLCasadiMath::MX_Xd_SubMatrix val);
#endif

    ///
    /// \brief Read and evaluate an equation
    /// \param result The number read (output)
    /// \param variables The variable set
    /// \return True on success
    ///
    bool read(
        double& result,
        const std::map<Equation, double> &variables);
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Read and evaluate an equation
    /// \param result The number read (output)
    /// \param variables The variable set
    /// \return True on success
    ///
    bool read(
        RBDLCasadiMath::MX_Xd_SubMatrix result,
        const std::map<Equation, double> &variables);
#endif

    ///
    /// \brief Read a certain amounts of bits in binary
    /// \param output Where the result is store
    /// \param n_elements Number of bits to read
    /// \return True on success
    ///
    bool readFromBinary(
        char* output,
        int n_elements);

    ///
    /// \brief Read a float in binary file
    /// \param result The float to put the result into
    /// \return True on success
    ///
    bool readFromBinary(
        float& result);

    ///
    /// \brief Read a float in binary file
    /// \return True on success
    ///
    bool readFromBinary(
        Vector3d& result);

    ///
    /// \brief Advance in the file to a specific tag
    /// \param tag The tag to reach
    /// \param text The text that follows a tag
    /// \return True on success
    ///
    bool readSpecificTag(
        const String& tag,
        String& text);

    ///
    /// \brief Advance in the file to a specific tag
    /// \param tag The tag to reach
    /// \param maxTag The number of element to read before giving up
    /// \return True on success
    ///
    bool reachSpecificTag(
        const String& tag,
        size_t maxTag = -1);

    ///
    /// \brief Counts the number of consecutive lines starting with the same tag and then brings it back to the initial position
    /// \param tag The tag to count
    /// \return The number of consecutive lines starting with the same tag
    ///
    int countTagsInAConsecutiveLines(
        const String& tag);

    ///
    /// \brief Read a whole line
    /// \param text The text read (output)
    ///
    void getline(
        String& text);

    ///
    /// \brief Reset the file cursor to the 0 position
    ///
    void resetCursor();

    ///
    /// \brief Close the file
    ///
    bool close();

    ///
    /// \brief Return if the file is at the end
    /// \return If the file is at the end
    ///
    bool eof();

protected:
    std::shared_ptr<bool> m_isOpen;///< If file is open
    char m_floatBuffer[sizeof(float)]; ///< Buffer for reading float in binaries

private:
    std::shared_ptr<std::ifstream> m_ifs;///< the ifstream
    std::shared_ptr<Path> m_path;///< The path of the file
};

}
}

#endif // BIORBD_UTILS_IFSTREAM_H

