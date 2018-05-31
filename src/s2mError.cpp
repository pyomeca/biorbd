#define BIORBD_API_EXPORTS
#include "../include/s2mError.h"

void s2mError::s2mAssert(bool cond, const s2mString &s){
    if (!cond){
        std::cout << "ERROR: " << s << std::endl;
        throw s;
    }

}

void s2mError::s2mWarning(bool cond, const s2mString &s){
    if (!cond){
        std::cout << "Warning: " << s << std::endl;
    }
}
