#ifndef __PROCESSINPUT_HPP__
#define __PROCESSINPUT_HPP__
// processInput must be included by a function which include <Python.h> and define pyoError
// The functions to get data are heavily inspired by the example available at this address:
// http://notes.secretsauce.net/notes/2017/07/23_interfacing-numpy-and-c-an-example.html

#include <string>
#include <sstream>

// Provide a to_string for any number
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

// Get the markers from a Markers3d array
int getMarkers(s2mMusculoSkeletalModel* m, PyObject* marks_in, std::vector<s2mMarkers> &marks_out){
    unsigned int nbMarkers = m->nTags();

    // Get dimensions of the data
    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)marks_in);
    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)marks_in);

    // Dimension controls
    if (ndim != 3){
        PyErr_SetString(pyoError, "Markers should be a FrameDependentNpArray");
        return 0;
    }

    if (dims[0] != 4){
        PyErr_SetString(pyoError, "Markers should have 4 rows (XYZ1)");
        return 0;
    }

    if (dims[1] != nbMarkers){
        PyErr_SetString(pyoError, std::string("Number of markers should fit the number of markers in the model (" + patch::to_string(nbMarkers) + ")").c_str());
        return 0;
    }

    // Get the data
    __attribute__((unused)) void*      data0    = PyArray_DATA    ((PyArrayObject*)marks_in);
    int nFrames = dims[2];

    // Dispatch the data into properly formatted classes
    marks_out = std::vector<s2mMarkers>(nFrames);

    for (int f = 0; f < nFrames; ++f){
        marks_out[f] = s2mMarkers();
        for (unsigned int m = 0; m<nbMarkers; ++m){
            double x(((double*)data0)[ 0*dims[2]*dims[1] + m*dims[2] + f]);
            double y(((double*)data0)[ 1*dims[2]*dims[1] + m*dims[2] + f]);
            double z(((double*)data0)[ 2*dims[2]*dims[1] + m*dims[2] + f]);
            marks_out[f].addMarker(Eigen::Vector3d(x, y, z));
        }
    }

    // Return
    return 1;
}

// Get generalized coordinates from a GeneralizedCoordinate array
int getGenCoord(s2mMusculoSkeletalModel* m, PyObject* Q_in, std::vector<s2mGenCoord> &Q_out){
    unsigned int nQ = m->nbQ();

    // Get dimensions of the data
    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)Q_in);
    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)Q_in);

    // Dimension controls
    if (ndim > 3){
        PyErr_SetString(pyoError, "GeneralizedCoordinate should be a FrameDependentNpArray");
        return 0;
    }

    if (dims[0] != nQ){
        PyErr_SetString(pyoError, std::string("The model requires " + patch::to_string(nQ) + " values for the generalized coordinates").c_str());
        return 0;
    }

    if (dims[1] != 1){
        PyErr_SetString(pyoError, "GeneralizedCoordinate must have exactly 1 column");
        return 0;
    }

    // Get the data
    __attribute__((unused)) void*      data0    = PyArray_DATA    ((PyArrayObject*)Q_in);
    int nFrames = -1;
    if (ndim == 2)
        nFrames = 1;
    else
        nFrames = dims[2];

    // Dispatch the data into properly formatted classes
    Q_out = std::vector<s2mGenCoord>(nFrames);

    for (int f = 0; f < nFrames; ++f){
        Q_out[f] = s2mGenCoord(nQ);
        for (unsigned int q = 0; q<nQ; ++q){
            Q_out[f][q] = ((double*)data0)[ q*nFrames + f];
        }
    }

    // Return
    return 1;
}

// __PROCESSINPUT_HPP__
#endif