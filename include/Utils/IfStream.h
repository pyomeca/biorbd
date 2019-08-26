#ifndef BIORBD_UTILS_IFSTREAM_H
#define BIORBD_UTILS_IFSTREAM_H

#include <map>
#include "biorbdConfig.h"
#include "Utils/Path.h"
#include <iostream>

namespace biorbd {
namespace utils {
class Equation;

class BIORBD_API IfStream
{
public:
    IfStream(
            const biorbd::utils::Path& path,
            std::ios_base::openmode mode );
    IfStream(
            const char* path,
            std::ios_base::openmode mode );
    IfStream( );
    virtual ~IfStream();

    bool open(
            const biorbd::utils::Path& path,
            std::ios_base::openmode mode );

    bool read(biorbd::utils::String&);
    bool readIgnoreCommentedLine(biorbd::utils::String&);
    bool read(int&);
    bool read(bool&);
    bool read(unsigned int&);
    bool read(double&);
    bool read(
            double&,
            const std::map<biorbd::utils::Equation, double> &);
    bool readSpecificTag(
            const biorbd::utils::String&,
            biorbd::utils::String&);
    bool reachSpecificTag(const biorbd::utils::String&); // Se déplace dans un fichier jusqu'à un tag donné, retourne le nombre de lignes sautées
    int countTagsInAConsecutiveLines(const biorbd::utils::String&); // Compte le nombre de ligne commençant par le même tag puis ramène le flux à l'endroit du début
    void getline(biorbd::utils::String&);
    bool close();

    bool eof();

protected:
    bool m_isOpen;

private:
    std::ifstream *m_ifs;
    biorbd::utils::Path m_path;
};

}}

#endif // BIORBD_UTILS_IFSTREAM_H

