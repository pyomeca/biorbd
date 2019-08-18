#ifndef BIORBD_MUSCLES_PATH_CHANGERS_H
#define BIORBD_MUSCLES_PATH_CHANGERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd {
namespace muscles {
class PathChanger;

class BIORBD_API PathChangers
{
public:
    PathChangers();
    virtual ~PathChangers();
    void addPathChanger(biorbd::muscles::PathChanger&); // Ajouter un wrapping ou un via point

    // Set and get
    unsigned int nbWraps() const; // retourne le nombre total de wrapping objects
    unsigned int nbVia() const; // retourne le nombre total de via points
    unsigned int nbObjects() const; // Retourne le nombre total d'objects
    const std::shared_ptr<biorbd::muscles::PathChanger> object(unsigned int  idx) const; // Get sur un wrapping

protected:
    std::vector<std::shared_ptr<biorbd::muscles::PathChanger>> m_obj; // Tableau de pointeur sur les objects
    unsigned int m_nbWraps;
    unsigned int m_nbVia;
    unsigned int m_totalObjects;

};

}}

#endif // BIORBD_MUSCLES_PATH_CHANGERS_H
