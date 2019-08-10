#define BIORBD_API_EXPORTS
#include "Utils/Error.h"

#include "Utils/String.h"

void s2mError::s2mAssert(bool cond, const s2mString &s){
    if (!cond){
        throw std::runtime_error(s);
    }

}

void s2mError::s2mWarning(bool cond, const s2mString &s){
    if (!cond){
        std::cout << "Warning: " << s << std::endl;
    }
}
