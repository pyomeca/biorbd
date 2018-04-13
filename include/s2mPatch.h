 
#ifndef S2MPATCH_H
#define S2MPATCH_H
    #include "vector"
    #include "Eigen/Dense"

class s2mPatch
{
    public:
        s2mPatch(const Eigen::Vector3i& = Eigen::Vector3i());

        // Ajouter un patch au lot
        int &operator() (int);
        void patch(const Eigen::Vector3i&);
        void patch(const s2mPatch&);
        s2mPatch patch(); // retourne les patchs
    protected:
        Eigen::Vector3i m_patch;
    private:
};

#endif // S2MPATCH_H
