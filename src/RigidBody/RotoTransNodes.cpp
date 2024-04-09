#define BIORBD_API_EXPORTS
#include "RigidBody/RotoTransNodes.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "Utils/Rotation.h"
#include "Utils/RotoTransNode.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Segment.h"

using namespace BIORBD_NAMESPACE;

rigidbody::RotoTransNodes::RotoTransNodes() :
    m_RTs(std::make_shared<std::vector<utils::RotoTransNode>>())
{
    //ctor
}

rigidbody::RotoTransNodes::RotoTransNodes(const
        rigidbody::RotoTransNodes &other)
{
    m_RTs = other.m_RTs;
}

rigidbody::RotoTransNodes::~RotoTransNodes()
{

}

rigidbody::RotoTransNodes rigidbody::RotoTransNodes::DeepCopy()
const
{
    rigidbody::RotoTransNodes copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::RotoTransNodes::DeepCopy(const
        rigidbody::RotoTransNodes &other)
{
    m_RTs->resize(other.m_RTs->size());
    for (size_t i=0; i<other.m_RTs->size(); ++i) {
        (*m_RTs)[i] = (*other.m_RTs)[i].DeepCopy();
    }
}

void rigidbody::RotoTransNodes::addRT()
{
    m_RTs->push_back(utils::RotoTransNode());
}

// Add a new marker to the existing pool of markers
void rigidbody::RotoTransNodes::addRT(
    const utils::RotoTransNode &RotoTrans)
{
    m_RTs->push_back(utils::RotoTransNode(RotoTrans));
}

size_t rigidbody::RotoTransNodes::nbRTs() const
{
    return m_RTs->size();
}

size_t rigidbody::RotoTransNodes::size() const
{
    return m_RTs->size();
}

// Get the markers in the global reference
const std::vector<utils::RotoTransNode>&
rigidbody::RotoTransNodes::RTs() const
{
    return *m_RTs;
}

std::vector<utils::RotoTransNode>
rigidbody::RotoTransNodes::RTs(
    const utils::String& segmentName)
{
    std::vector<utils::RotoTransNode> pos;
    for (size_t i=0; i<nbRTs();
            ++i) // Scan through all the markers and select the good ones
        if (!RT(i).parent().compare(segmentName)) {
            pos.push_back(RT(i));
        }
    return pos;
}

const utils::RotoTransNode& rigidbody::RotoTransNodes::RT(
    size_t idx)
{
    return (*m_RTs)[idx];
}

// Get the RotoTransNodes at the position given by Q
std::vector<utils::RotoTransNode>
rigidbody::RotoTransNodes::RTs(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<utils::RotoTransNode> pos;
    for (size_t i=0; i<nbRTs(); ++i) {
        pos.push_back(RT(Q, i, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get an IMU at the position given by Q
utils::RotoTransNode rigidbody::RotoTransNodes::RT(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    utils::RotoTransNode node = RT(idx);
    return model.globalJCS(Q, node.parent(), updateKin) * node;
}

std::vector<utils::RotoTransNode>
rigidbody::RotoTransNodes::segmentRTs(
    const rigidbody::GeneralizedCoordinates &Q,
    size_t idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Segment name to find
    utils::String name(model.segment(idx).name());

    std::vector<utils::RotoTransNode> pos;
    for (size_t i=0; i<nbRTs();
            ++i) // scan all the markers and select the right ones
        if (!((*m_RTs)[i]).parent().compare(name)) {
            pos.push_back(RT(Q,i,updateKin));
            updateKin = false;
        }

    return pos;
}

// Get the Jacobian of the markers
std::vector<utils::Matrix>
rigidbody::RotoTransNodes::RTsJacobian(
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    std::vector<utils::Matrix> G;

    for (size_t idx=0; idx<nbRTs(); ++idx) {
        // Actual marker
        utils::RotoTransNode node = RT(idx);

        size_t id = model.GetBodyId(node.parent().c_str());
        utils::Matrix G_tp(utils::Matrix::Zero(9,model.dof_count));

        // Calculate the Jacobian of this Tag
        model.CalcMatRotJacobian(Q, id, node.rot(), G_tp, updateKin); 
        updateKin = false; // False for speed

        G.push_back(G_tp);
    }

    return G;
}

std::vector<utils::String> rigidbody::RotoTransNodes::RTsNames()
{
    // Extract the name of all the markers of a model
    std::vector<utils::String> names;
    for (size_t i=0; i<nbRTs(); ++i) {
        names.push_back(RT(i).utils::Node::name());
    }
    return names;
}

void rigidbody::RotoTransNodes::clear() {
    m_RTs->clear();
}