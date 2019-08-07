#include <iostream>
#include <gtest/gtest.h>

#include "s2mMusculoSkeletalModel.h"

TEST(c3dFileIO, OpenModel){
    EXPECT_NO_THROW(s2mMusculoSkeletalModel model("models/pyomecaman.bioMod"));
}
