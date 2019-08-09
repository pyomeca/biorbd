#include <iostream>
#include <gtest/gtest.h>

#include "s2mMusculoSkeletalModel.h"

static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
    EXPECT_NO_THROW(s2mMusculoSkeletalModel model(modelPathForGeneralTesting));
TEST(c3dFileIO, OpenModel){
}

static std::string modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
TEST(c3dConstraint, loopConstraint){
    s2mMusculoSkeletalModel model(modelPathForLoopConstraintTesting);

    s2mContacts& cs(model.getConstraints(model));
    std::cout << cs.nContacts() << std::endl;
}
