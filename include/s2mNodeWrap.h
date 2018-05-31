#ifndef S2MNODEWRAP_H
#define S2MNODEWRAP_H
#include <Eigen/Dense>
#include "s2mNode.h"


class s2mNode;
class s2mNodeWrap : public s2mNode
{
    public:
        s2mNodeWrap(const Eigen::Vector3d& = Eigen::Vector3d(0,0,0), // Position
                      const s2mString& = "", // Nom du noeud
                      const s2mString& = ""); //  Nom du parent

        virtual ~s2mNodeWrap();
        // Get and Set

    protected:
    private:

};

#endif // S2MNODEWRAP_H
