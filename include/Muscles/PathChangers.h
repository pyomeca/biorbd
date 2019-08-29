#ifndef BIORBD_MUSCLES_PATH_CHANGERS_H
#define BIORBD_MUSCLES_PATH_CHANGERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Node3d;
}

namespace muscles {

class BIORBD_API PathChangers
{
public:
    PathChangers();
    biorbd::muscles::PathChangers DeepCopy() const;
    void DeepCopy(const biorbd::muscles::PathChangers& other);

    void addPathChanger(biorbd::utils::Node3d&); // Ajouter un wrapping ou un via point

    // Set and get
    unsigned int nbWraps() const; // retourne le nombre total de wrapping objects
    unsigned int nbVia() const; // retourne le nombre total de via points
    unsigned int nbObjects() const; // Retourne le nombre total d'objects
    biorbd::utils::Node3d& object(unsigned int  idx); // Get sur un wrapping
    const biorbd::utils::Node3d& object(unsigned int  idx) const; // Get sur un wrapping

protected:
    std::shared_ptr<std::vector<biorbd::utils::Node3d>> m_obj; // Tableau de pointeur sur les objects
    std::shared_ptr<unsigned int> m_nbWraps;
    std::shared_ptr<unsigned int> m_nbVia;
    std::shared_ptr<unsigned int> m_totalObjects;

};

}}

#endif // BIORBD_MUSCLES_PATH_CHANGERS_H
