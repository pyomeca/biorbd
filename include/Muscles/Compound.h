#ifndef BIORBD_MUSCLES_COMPOUND_H
#define BIORBD_MUSCLES_COMPOUND_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Utils/GenCoord.h"
#include "Muscles/Force.h"
#include "Muscles/Geometry.h"
#include "Muscles/Caracteristics.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/PathChangers.h"

namespace biorbd { namespace  muscles {

class BIORBD_API Compound
{
public:
    Compound(
            const biorbd::utils::String &name = "",
            const biorbd::muscles::PathChangers& = biorbd::muscles::PathChangers());
    Compound(const Compound& m);
    virtual ~Compound();

    // Wrapping object
    const biorbd::muscles::PathChangers& pathChanger();
    void addPathObject(biorbd::muscles::PathChanger &w); // Ajouter un wrapping object


    const biorbd::utils::String& type() const;
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            s2mJoints& model,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            const int updateKin = 2) = 0;
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            s2mJoints& model,
            const biorbd::utils::GenCoord& Q,
            const biorbd::muscles::StateDynamics& emg,
            const int updateKin = 2) = 0;
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(const biorbd::muscles::StateDynamics& emg) = 0;
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(); // Return the last computed muscle force

    const biorbd::utils::String& name() const;
    void setName(const biorbd::utils::String& name);
protected:
    biorbd::muscles::PathChangers m_pathChanger;
    std::vector<std::shared_ptr<biorbd::muscles::Force>> m_force;
    void copyForce(const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force);
    virtual void setForce() = 0;
    virtual void setType()=0;
    biorbd::utils::String m_type;
    biorbd::utils::String m_name;

};

}}

#endif // BIORBD_MUSCLES_COMPOUND_H
