#define BIORBD_API_EXPORTS
#include "Utils/Error.h"

#include "Utils/String.h"

void biorbd::utils::Error::error(bool cond, const biorbd::utils::String &s){
    if (!cond){
        throw std::runtime_error(s);
    }

}

void biorbd::utils::Error::warning(bool cond, const biorbd::utils::String &s){
    if (!cond){
        std::cout << "Warning: " << s << std::endl;
    }
}
