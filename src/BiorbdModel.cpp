#define BIORBD_API_EXPORTS
#include "BiorbdModel.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "ModelReader.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/NodeSegment.h"
#include "Utils/String.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

utils::String getVersion()
{
    return BIORBD_VERSION;
}

Model::Model() :
    m_path(std::make_shared<utils::Path>())
{

}

Model::Model(const utils::Path &path) :
    m_path(std::make_shared<utils::Path>(path))
{

    Reader::readModelFile(*m_path, this);
}

utils::Path Model::path() const
{
    return *m_path;
}
