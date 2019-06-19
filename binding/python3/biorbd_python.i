/* File : biorbd_python.i */
%{
#define SWIG_FILE_WITH_INIT
#include "s2mMusculoSkeletalModel.h"
#ifdef _WIN32
// This is a hack because Eigen can't be dynamically compiled on Windows, while dlib needs consistency in compilation. 
// Please note that this can result in undefined behavior while using s2mMuscleOptimisation...
const int USER_ERROR__inconsistent_build_configuration__see_dlib_faq_1_ = 0;
const int DLIB_VERSION_MISMATCH_CHECK__EXPECTED_VERSION_19_10_0 = 0;
#endif
#include "Python.h"
#include "numpy/arrayobject.h"
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
%include <std_string.i>

/*** s2mJoints ***/
%typemap(typecheck) s2mJoints &{
    void *argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mJoints,  0  | 0)) && argp1) {
        // Test if it is a pointer to SWIGTYPE_p_s2mJoints already exists
        $1 = true;
    } else {
        $1 = false;
    }
}
%typemap(in) s2mJoints &{
    void * argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mJoints,  0  | 0)) && argp1) {
        // Recast the pointer
        $1 = reinterpret_cast< s2mJoints * >(argp1);
    } else {
        PyErr_SetString(PyExc_ValueError, "s2mJoints must be a s2mJoints");
        SWIG_fail;
    }
}

/*** s2mMatrix ***/
%extend s2mMatrix{
    PyObject* get_array(){
        int nRows($self->rows());
        int nCols($self->cols());
        int nArraySize(2);
        npy_intp * arraySizes = new npy_intp[nArraySize];
        arraySizes[0] = nRows;
        arraySizes[1] = nCols;

        double * matrix = new double[nRows*nCols];
        unsigned int k(0);
        for (unsigned int i=0; i<nRows; ++i){
            for (unsigned int j=0; j<nCols; ++j){
                matrix[k] = (*$self)(i, j);
                ++k;
            }
        }
        PyObject* output = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, matrix);
        PyArray_ENABLEFLAGS((PyArrayObject *)output, NPY_ARRAY_OWNDATA);
        return output;
    };
}

/*** s2mVector ***/
%extend s2mVector{
    PyObject* get_array(){
        int nElements($self->size());
        int nArraySize(1);
        npy_intp * arraySizes = new npy_intp[nArraySize];
        arraySizes[0] = nElements;

        double * vect = new double[nElements];
        for (unsigned int i=0; i<nElements; ++i){
            vect[i] = (*$self)(i);
        }
        PyObject* output = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, vect);
        PyArray_ENABLEFLAGS((PyArrayObject *)output, NPY_ARRAY_OWNDATA);
        return output;
    };
}

/*** s2mGenCoord ***/
%typemap(typecheck) s2mGenCoord &{
    void *argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mGenCoord,  0  | 0)) && argp1) {
        // Test if it is a pointer to SWIGTYPE_p_s2mGenCoord already exists
        $1 = true;
    } else if( PyArray_Check($input) ) {
        // test if it is a numpy array
        $1 = true;
    } else {
        $1 = false;
    }
}
%typemap(in) s2mGenCoord &{
    void * argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mGenCoord,  0  | 0)) && argp1) {
        // Recast the pointer
        $1 = reinterpret_cast< s2mGenCoord * >(argp1);
    } else if( PyArray_Check($input) ) {
        // Get dimensions of the data
        int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
        npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

        // Dimension controls
        if (ndim != 1 ){
            PyErr_SetString(PyExc_ValueError, "s2mGenCoord must be a numpy vector");
            SWIG_fail;
        }

        // Cast the vector
        PyObject *data = PyArray_FROM_OTF((PyObject*)$input, NPY_DOUBLE, NPY_IN_ARRAY);
        // Copy the actual data
        unsigned int nQ(dims[0]);
        $1 = new s2mGenCoord(nQ);
        for (unsigned int q=0; q<nQ; ++q)
            (*$1)[q] = *(double*)PyArray_GETPTR1(data, q);

    } else {
        PyErr_SetString(PyExc_ValueError, "s2mGenCoord must be a s2mGenCoord or numpy vector");
        SWIG_fail;
    }
};

/*** s2mTau ***/
%typemap(typecheck) s2mTau &{
    void *argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mTau,  0  | 0)) && argp1) {
        // Test if it is a pointer to SWIGTYPE_p_s2mTau already exists
        $1 = true;
    } else if( PyArray_Check($input) ) {
        // test if it is a numpy array
        $1 = true;
    } else {
        $1 = false;
    }
}
%typemap(in) s2mTau &{
    void * argp1 = 0;
        if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mTau,  0  | 0)) && argp1) {
        // If it is the right type, recast-it the pointer
        $1 = reinterpret_cast< s2mTau * >(argp1);
    } else if( PyArray_Check($input) ) {
        // Get dimensions of the data
        int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
        npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

        // Dimension controls (has to be a vector)
        if (ndim != 1 ){
            PyErr_SetString(PyExc_ValueError, "s2mTau must be a numpy vector");
            SWIG_fail;
        }

        // Cast the vector
        PyObject *data = PyArray_FROM_OTF((PyObject*)$input, NPY_DOUBLE, NPY_IN_ARRAY);

        // Copy the actual data
        unsigned int nTau(dims[0]);
        $1 = new s2mTau(nTau);
        for (unsigned int tau=0; tau<nTau; ++tau)
            (*$1)[tau] = *(double*)PyArray_GETPTR1(data, tau);

    } else {
        PyErr_SetString(PyExc_ValueError, "s2mTau must be a s2mTau or numpy vector");
        SWIG_fail;
    }
};


/*** s2mMarkers ***/
%typemap(typecheck) s2mMarkers &{
    void *argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mMarkers,  0  | 0)) && argp1) {
        // Test if it is a pointer to SWIGTYPE_p_s2mMarkers already exists
        $1 = true;
    } else {
        $1 = false;
    }
}
%typemap(in) s2mMarkers &{
    void * argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mMarkers,  0  | 0)) && argp1) {
        // Recast the pointer
        $1 = reinterpret_cast< s2mMarkers * >(argp1);
    } else {
        PyErr_SetString(PyExc_ValueError, "s2mMarkers must be a s2mMarkers");
        SWIG_fail;
    }
}


/*** s2mNode ***/
%typemap(typecheck) s2mNode &{
    void *argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mNode,  0  | 0)) && argp1) {
        // Test if it is a pointer to SWIGTYPE_p_s2mNode already exists
        $1 = true;
    } else if( PyArray_Check($input) ) {
        // test if it is a numpy array
        $1 = true;
    } else {
        $1 = false;
    }
}
%typemap(in) s2mNode &{
    void * argp1 = 0;
    if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp1, SWIGTYPE_p_s2mNode,  0  | 0)) && argp1) {
        // Recast the pointer
        $1 = reinterpret_cast< s2mNode * >(argp1);
    } else if( PyArray_Check($input) ) {
        // Get dimensions of the data
        int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
        npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

        // Dimension controls
        if (ndim != 1 && (dims[0] < 3 || dims[0] > 4)){
            PyErr_SetString(PyExc_ValueError, "s2mNode must be a numpy 3d vector");
            SWIG_fail;
        }
        // Cast the vector
        PyObject *data = PyArray_FROM_OTF((PyObject*)$input, NPY_DOUBLE, NPY_IN_ARRAY);

        // Copy the actual data
        $1 = new s2mNode();
        for (unsigned int i=0; i<3; ++i)
            (*$1)[i] = *(double*)PyArray_GETPTR1(data, i);

    } else {
        PyErr_SetString(PyExc_ValueError, "s2mNode must be a s2mNode or numpy vector");
        SWIG_fail;
    }
};
%extend s2mNode{
    PyObject* get_array(){
        int nArraySize(1);
        npy_intp * arraySizes = new npy_intp[nArraySize];
        arraySizes[0] = 3;

        double * node = new double[3];
        for (unsigned int i=0; i<3; ++i){
            node[i] = (*$self)(i);
        }
        PyObject* output = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, node);
        PyArray_ENABLEFLAGS((PyArrayObject *)output, NPY_ARRAY_OWNDATA);
        return output;
    }
};


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
    } else {
        PyErr_SetString(PyExc_ValueError, "s2mPath must be a s2mPath or string");
        SWIG_fail;
    }
};

%extend s2mAttitude{
    PyObject* get_array(){
        int nArraySize(2);
        npy_intp * arraySizes = new npy_intp[nArraySize];
        arraySizes[0] = 4;
        arraySizes[1] = 4;

        double * values = new double[4*4];
        for (unsigned int i=0; i<4; ++i){
            for (unsigned int j=0; j<4; ++j){
                values[i*4+j] = (*$self)(j*4+i);
            }
        }
        PyObject* output = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, values);
        PyArray_ENABLEFLAGS((PyArrayObject *)output, NPY_ARRAY_OWNDATA);
        return output;
    }
};

%include ../biorbd.i



