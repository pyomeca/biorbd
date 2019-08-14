#ifndef S2M_IMU_UNITY_OPTIM_H
#define S2M_IMU_UNITY_OPTIM_H

#include <dlib/optimization.h>
#include "biorbdConfig.h"
#include "Utils/Attitude.h"

class BIORBD_API s2mIMU_Unity_Optim
{
public:
    typedef dlib::matrix<double,1,1> parameter_vector;
    class OptimData{
    public:
        OptimData(
                const biorbd::utils::Attitude &R1,
                const biorbd::utils::Attitude &R2, int axe);
        biorbd::utils::Attitude m_R1;
        biorbd::utils::Attitude m_R2;
        int m_axe;
    };
    static biorbd::utils::Attitude alignSpecificAxisWithParentVertical(
            const biorbd::utils::Attitude &r1,
            const biorbd::utils::Attitude &r2,
            int idxAxe); // Optimization

protected:
    static double residual (
            const OptimData& data,
            const parameter_vector& x); // Optimization

};

#endif // S2M_IMU_UNITY_OPTIM_H
