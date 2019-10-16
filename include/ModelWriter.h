#ifndef BIORBD_UTILS_WRITER_H
#define BIORBD_UTILS_WRITER_H

#include "biorbdConfig.h"
///
/// \brief Namespace that holds the whole model
///
namespace biorbd {
class Model;

///
/// \brief Namespace that holds the path
///
namespace utils {
class Path;
}

///
/// \brief Main class for writer
///
class BIORBD_API Writer
{
public:

///
/// \brief Writes the model
/// \param model The model
/// \param pathToWrite The path to write (folder in which to write)
///
    static void writeModel(
            biorbd::Model &model,
            const biorbd::utils::Path& pathToWrite);

};

}

#endif // BIORBD_UTILS_WRITER_H
