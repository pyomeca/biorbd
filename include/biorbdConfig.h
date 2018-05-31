#ifndef __BIORBD_CONFIG_H__
#define __BIORBD_CONFIG_H__

#ifdef _WIN32
#  ifdef BIORBD_API_EXPORTS
#    define BIORBD_API __declspec(dllexport)
#  else
#    define BIORBD_API __declspec(dllimport)
#  endif
#else
#  define BIORBD_API
#endif

#endif // __BIORBD_CONFIG_H__
