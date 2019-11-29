#define BIORBD_API_EXPORTS
#include "BiorbdModel.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "ModelReader.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/NodeSegment.h"

biorbd::Model::Model()
{

}

biorbd::Model::~Model()
{

}

biorbd::Model::Model(const biorbd::utils::Path &path)
{
    biorbd::Reader::readModelFile(path, this);
}
