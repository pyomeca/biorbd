 
#ifndef S2MBONEMESH_H
#define S2MBONEMESH_H
    #include "biorbdConfig.h"
    #include "s2mNode.h"
    #include "s2mPatch.h"
    #include "s2mPath.h"
    #include "vector"

class BIORBD_API s2mBoneMesh
{
    public:
        s2mBoneMesh(
                const std::vector<s2mNode>& = std::vector<s2mNode>(),
                const std::vector<s2mPatch>& = std::vector<s2mPatch>());
        virtual ~s2mBoneMesh();

        // Concernant les points du mesh
        void addPoint(const s2mNode &);
        const s2mNode& point(unsigned int i) const;
        unsigned int size() const;

        void setPath(const s2mPath& path);
        const s2mPath& path() const;

        // Concernant les patch
        void addPatch(const s2mPatch &);
        void addPatch(const Eigen::Vector3i &);
        const std::vector<s2mPatch>& patch() const;
        const s2mPatch& patch(unsigned int i) const;
        unsigned int sizePatch();

    protected:
        s2mPath m_pathFile;
        std::vector<s2mNode> m_mesh;
        std::vector<s2mPatch> m_patch;
    private:
};

#endif // S2MBONEMESH_H
