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


// Typemaps for all communication with BIORBD
//%typemap(in) s2mJoints &{
//    std::cout << "s2joints&" << std::endl;
//    $1 = reinterpret_cast<s2mJoints *>($1);
//}
//%typemap(in) s2mJoints{
//    std::cout << "s2joints" << std::endl;
//    $1 = reinterpret_cast<s2mJoints *>($1);
//}
//%typemap(out) s2mMusculoSkeletalModel &{
//    std::cout << "s2joints&_out" << std::endl;
//    $result = (PyObject *)$1;
////    Py_INCREF($1);
//}
//%typemap(out) s2mMusculoSkeletalModel{
//    std::cout << "s2joints_out" << std::endl;
//    PyArg_ParseTuple(args, "l", &address_m)
//    $result = &$1;
////    Py_INCREF($1);
//}

/*** s2mGenCoord ***/
%typemap(in) s2mGenCoord &{
    // Get dimensions of the data
    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

    // Dimension controls
    if (ndim != 1 ){
        PyErr_SetString(PyExc_ValueError, "s2mGenCoord must be a numpy vector");
        return 0;
    }

    // Get the data
    __attribute__((unused)) void*      data    = PyArray_DATA    ((PyArrayObject*)$input);

    unsigned int nQ(dims[0]);
    $1 = new s2mGenCoord(nQ);
    for (unsigned int q = 0; q<nQ; ++q){
        (*$1)[q] = ((double*)data)[q];
    }
};

%typemap(out) s2mGenCoord{
    int nQ($1.size());
    int nArraySize(1);
    npy_intp * arraySizes = new npy_intp[nArraySize];
    arraySizes[0] = nQ;

    double * q = new double[nQ];
    for (unsigned int i=0; i<nQ; ++i){
        q[i] = $1(i);
    }
    $result = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, q);
    PyArray_ENABLEFLAGS((PyArrayObject *)$result, NPY_ARRAY_OWNDATA);
};


/*** s2mNode ***/
%typemap(in) s2mNode &{
    // Get dimensions of the data
    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

    // Dimension controls
    if (ndim != 1 && (dims[0] < 3 || dims[0] > 4)){
        PyErr_SetString(PyExc_ValueError, "s2mNode must be a numpy 3d vector");
        return 0;
    }

    // Get the data
    __attribute__((unused)) void*      data    = PyArray_DATA    ((PyArrayObject*)$input);

    $1 = new s2mNode();
    for (unsigned int i = 0; i<3; ++i){
        (*$1)[i] = ((double*)data)[i];
    }
};

%typemap(out) s2mNode{
    int nArraySize(1);
    npy_intp * arraySizes = new npy_intp[nArraySize];
    arraySizes[0] = 3;

    double * node = new double[3];
    for (unsigned int i=0; i<3; ++i){
        node[i] = $1(i);
    }
    $result = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, node);
    PyArray_ENABLEFLAGS((PyArrayObject *)$result, NPY_ARRAY_OWNDATA);
};

/*** s2mNode ***/
%typemap(in) s2mNodeBone &{
    // Get dimensions of the data
    __attribute__((unused)) int        ndim     = PyArray_NDIM    ((PyArrayObject*)$input);
    __attribute__((unused)) npy_intp*  dims     = PyArray_DIMS    ((PyArrayObject*)$input);

    // Dimension controls
    if (ndim != 1 && (dims[0] < 3 || dims[0] > 4)){
        PyErr_SetString(PyExc_ValueError, "s2mNode must be a numpy 3d vector");
        return 0;
    }

    // Get the data
    __attribute__((unused)) void*      data    = PyArray_DATA    ((PyArrayObject*)$input);

    $1 = new s2mNodeBone();
    for (unsigned int i = 0; i<3; ++i){
        (*$1)[i] = ((double*)data)[i];
    }
};

%typemap(out) s2mNodeBone{
    int nArraySize(1);
    npy_intp * arraySizes = new npy_intp[nArraySize];
    arraySizes[0] = 3;

    double * node = new double[3];
    for (unsigned int i=0; i<3; ++i){
        node[i] = $1(i);
    }
    $result = PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, node);
    PyArray_ENABLEFLAGS((PyArrayObject *)$result, NPY_ARRAY_OWNDATA);
};


/*** s2mPath ***/
////    std::cout << "$input" << std::endl;
////    std::cout << $input << std::endl;
////    //$1 = new s2mPath(PyUnicode_AsUTF8($input));
////    $1 = reinterpret_cast<s2mPath *>($input);
////    std::cout << "$1" << std::endl;
////    std::cout << "end" << std::endl;
//    std::cout << "end" << std::endl;
//    PyObject * tp;
//    PyArg_ParseTuple($input, "O", tp);
//    std::cout << "end" << std::endl;
//    $1 = reinterpret_cast<s2mPath *>(tp);
//    std::cout << "end" << std::endl;
//        std::cout << $1->filename() << std::endl;
//            std::cout << "end" << std::endl;
//    PyObject * tp;
//    PyArg_ParseTuple($input, "O", tp);
//    $1 = new s2mPath(*(reinterpret_cast<s2mPath*>(tp)));

%typemap(in) s2mPath &{
    void *argp1 = 0 ;
    int res1 = SWIG_ConvertPtr(obj0, &argp1, SWIGTYPE_p_s2mPath,  0  | 0);
    if (!SWIG_IsOK(res1)) {
        SWIG_exception_fail(SWIG_ArgError(res1), "in method '" "new_s2mMusculoSkeletalModel" "', argument " "1"" of type '" "s2mPath const &""'");
    }
    if (!argp1) {
        SWIG_exception_fail(SWIG_ValueError, "invalid null reference " "in method '" "new_s2mMusculoSkeletalModel" "', argument " "1"" of type '" "s2mPath const &""'");
      }
    $1 = reinterpret_cast< s2mPath * >(argp1);
};
%typemap(in) s2mPath &{
    $1 = new s2mPath(PyUnicode_AsUTF8($input));
};


%include ../biorbd.i



