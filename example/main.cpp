#include <iostream>
#include <memory>
#include <cstdlib>
#include <random>

#include "all.h"
//#define BUILD_SANDBOX

#ifndef BUILD_SANDBOX

int main()
{
    // One can look at the tests to find more in-depth examples
    biorbd::Model m("pyomecaman.bioMod");
    return 0;
}

#else
int main()
{
    // Anything you want to quickly test can be tested here assuming that you
    // uncommented the "#define BUILD_SANDBOX" line at the beginning of the file
    return 0;
}
#endif
