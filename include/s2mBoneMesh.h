 
#ifndef S2MBONEMESH_H
#define S2MBONEMESH_H
    #include "s2mNode.h"
    #include "s2mPatch.h"
    #include "s2mPath.h"
    #include "vector"

class s2mBoneMesh
{
    public:
        s2mBoneMesh(const std::vector<s2mNode>& = std::vector<s2mNode>(), const std::vector<s2mPatch>& = std::vector<s2mPatch>());
        ~s2mBoneMesh();

        // Concernant les points du mesh
        void addPoint(const s2mNode &);
        s2mNode point(unsigned int i);
        unsigned int size();

        void setPath(const s2mPath& path);
        s2mPath path() const;

        // Concernant les patch
        void addPatch(const s2mPatch &);
        void addPatch(const Eigen::Vector3i &);
        const std::vector<s2mPatch>& patch();
        s2mPatch patch(unsigned int i);
        unsigned int sizePatch();

    protected:
        s2mPath m_pathFile;
        std::vector<s2mNode> m_mesh;
        std::vector<s2mPatch> m_patch;
    private:
};

#endif // S2MBONEMESH_H
