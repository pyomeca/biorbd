/* File : biorbd_python.i */
%{
#define SWIG_FILE_WITH_INIT
#include "s2mMusculoSkeletalModel.h"
%}

%include "numpy.i"
%init %{
    import_array();
%}


// typemaps.i is a built-in swig interface that lets us map c++ types to other
// types in our language of choice. We'll use it to map Eigen matrices to
// Numpy arrays.
%include <typemaps.i>
%include <std_vector.i>

///*** s2mGenCoord ***/
//%typemap(typecheck) s2mGenCoord &{
//    void *argp1 = 0;
//    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mGenCoord,  0  | 0)) && argp1) {
//        // Test if it is a pointer to s2mGenCoord already exists
//        $1 = true;
//    } else if( PyUnicode_Check($input) ) {
//        // test if it is a string (python3 has unicode)
//        $1 = true;
//    } else {
//        $1 = false;
//    }
//}
//%typemap(in) s2mGenCoord &{
//    // BENJAMIN 1
//    // Get dimensions of the data
//    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
//    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

//    // Dimension controls
//    if (ndim != 1 ){
//        PyErr_SetString(PyExc_ValueError, "s2mGenCoord must be a numpy vector");
//        return 0;
//    }

//    // Get the data
//    __attribute__((unused)) void*      data    = PyArray_DATA    ((PyArrayObject*)$input);

//    unsigned int nQ(dims[0]);
//    $1 = new s2mGenCoord(nQ);
//    for (unsigned int q = 0; q<nQ; ++q){
//        (*$1)[q] = ((double*)data)[q];
//    }
//};

//%typemap(out) s2mGenCoord{
//    int nQ($1.size());
//    int nArraySize(1);
//    npy_intp * arraySizes = new npy_intp[nArraySize];
//    arraySizes[0] = nQ;

//    double * q = new double[nQ];
//    for (unsigned int i=0; i<nQ; ++i){
//        q[i] = $1(i);
//    }
//    $result = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, q);
//    PyArray_ENABLEFLAGS((PyArrayObject *)$result, NPY_ARRAY_OWNDATA);
//};


///*** s2mNode ***/
//%typemap(in) s2mNode &{
//    // Get dimensions of the data
//    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
//    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

//    // Dimension controls
//    if (ndim != 1 && (dims[0] < 3 || dims[0] > 4)){
//        PyErr_SetString(PyExc_ValueError, "s2mNode must be a numpy 3d vector");
//        return 0;
//    }

//    // Get the data
//    __attribute__((unused)) void*      data    = PyArray_DATA    ((PyArrayObject*)$input);

//    $1 = new s2mNode();
//    for (unsigned int i = 0; i<3; ++i){
//        (*$1)[i] = ((double*)data)[i];
//    }
//};

//%typemap(out) s2mNode{
//    int nArraySize(1);
//    npy_intp * arraySizes = new npy_intp[nArraySize];
//    arraySizes[0] = 3;

//    double * node = new double[3];
//    for (unsigned int i=0; i<3; ++i){
//        node[i] = $1(i);
//    }
//    $result = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, node);
//    PyArray_ENABLEFLAGS((PyArrayObject *)$result, NPY_ARRAY_OWNDATA);
//};

///*** s2mNode ***/
//%typemap(in) s2mNodeBone &{
//    // Get dimensions of the data
//    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
//    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

//    // Dimension controls
//    if (ndim != 1 && (dims[0] < 3 || dims[0] > 4)){
//        PyErr_SetString(PyExc_ValueError, "s2mNode must be a numpy 3d vector");
//        return 0;
//    }

//    // Get the data
//    __attribute__((unused)) void*      data    = PyArray_DATA    ((PyArrayObject*)$input);

//    $1 = new s2mNodeBone();
//    for (unsigned int i = 0; i<3; ++i){
//        (*$1)[i] = ((double*)data)[i];
//    }
//};

//%typemap(out) s2mNodeBone{
//    int nArraySize(1);
//    npy_intp * arraySizes = new npy_intp[nArraySize];
//    arraySizes[0] = 3;

//    double * node = new double[3];
//    for (unsigned int i=0; i<3; ++i){
//        node[i] = $1(i);
//    }
//    $result = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, node);
//    PyArray_ENABLEFLAGS((PyArrayObject *)$result, NPY_ARRAY_OWNDATA);
//};


/*** s2mPath ***/
%typemap(typecheck) s2mPath &{
    void *argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mPath,  0  | 0)) && argp1) {
        // Test if it is a pointer to s2mPath already exists
        $1 = true;
    } else if( PyUnicode_Check($input) ) {
        // test if it is a string (python3 has unicode)
        $1 = true;
    } else {
        $1 = false;
    }
};
%typemap(in) s2mPath &{
    void * argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mPath,  0  | 0)) && argp1) {
        // Recast the pointer
        $1 = reinterpret_cast< s2mPath * >(argp1);
    } else if( PyUnicode_Check($input) ) {
        // Interpret the string
        $1 = new s2mPath(PyUnicode_AsUTF8($input));
    }
};

%include ../biorbd.i



