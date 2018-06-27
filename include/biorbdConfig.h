#ifndef __BIORBD_CONFIG_H__
#define __BIORBD_CONFIG_H__
/*
Some day, if Eigen can be compile in DLL
#ifdef _WIN32
#  ifdef BIORBD_API_EXPORTS
#    define BIORBD_API __declspec(dllexport)
#  else
#    define BIORBD_API __declspec(dllimport)
#  endif
#else
#  define BIORBD_API
#endif
*/
#define BIORBD_API

#ifndef _WIN32
#ifndef NAN
    //static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
    #define NAN (*(const float *) __nan)
#endif // NAN
    
#endif // _WIN32
#endif // __BIORBD_CONFIG_H__