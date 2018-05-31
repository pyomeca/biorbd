/* File : biorbd.i */
%module biorbd
%{
#include "s2mMusculoSkeletalModel.h"
%}

/* Instantiate std_vector */
%include <std_vector.i>

/* Instantiate std_string */
%include <std_iostream.i>

// Instantiate templates
namespace std {

}


/* Includes all neceressary files from the API */
%include "s2mMusculoSkeletalModel.h"

