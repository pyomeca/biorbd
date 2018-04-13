#ifndef S2MMIFSTREAM_H
#define S2MMIFSTREAM_H

#include "s2mError.h"
#include <fstream>
#include "s2mEquation.h"
#include "s2mPath.h"
#include <boost/lexical_cast.hpp>
#include <vector>
#include <map>

class s2mIfStream
{
    public:
    s2mIfStream(const s2mPath& path, std::ios_base::openmode mode );
    s2mIfStream(const char* path, std::ios_base::openmode mode );
    s2mIfStream( );
    ~s2mIfStream();

    bool open(const s2mPath& path, std::ios_base::openmode mode );
    bool open(const char* path, std::ios_base::openmode mode  );

    bool read(s2mString&);
    bool readIgnoreCommentedLine(s2mString&);
    bool read(int&);
    bool read(bool&);
    bool read(unsigned int&);
    bool read(double&);
    bool read(double&, const std::map<s2mEquation, double> &);
    bool readSpecificTag(const s2mString&, s2mString&);
    bool reachSpecificTag(const s2mString&); // Se déplace dans un fichier jusqu'à un tag donné, retourne le nombre de lignes sautées
    int countTagsInAConsecutiveLines(const s2mString&); // Compte le nombre de ligne commençant par le même tag puis ramène le flux à l'endroit du début
    void getline(s2mString&);
    bool close();

    bool eof(){return m_ifs->eof();}

    protected:
        bool m_isOpen;


    private:
        std::ifstream *m_ifs;
        s2mPath m_path;
};

#endif // S2MMIFSTREAM_H

