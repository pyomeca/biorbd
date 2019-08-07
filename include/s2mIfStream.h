#ifndef S2MMIFSTREAM_H
#define S2MMIFSTREAM_H

#include <map>
#include "biorbdConfig.h"
#include "s2mPath.h"

class s2mEquation;
class BIORBD_API s2mIfStream
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

    bool eof();

    protected:
        bool m_isOpen;


    private:
        std::ifstream *m_ifs;
        s2mPath m_path;
};

#endif // S2MMIFSTREAM_H

