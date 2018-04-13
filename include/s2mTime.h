#ifndef S2MTIME_H
#define S2MTIME_H


class s2mTime
{
    public:
        s2mTime(const double &timeStep, const unsigned int &nbSteps);
        ~s2mTime();

        double time(const unsigned int &t){if (t>=m_nbSteps) return 0; else return m_time[t];  } // Return time at index t
    protected:
        double * m_time;
        unsigned int m_nbSteps;

    private:
};

#endif // S2MTIME_H
