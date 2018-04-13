#ifndef __PYORBDL_HPP__
#define __PYORBDL_HPP__

#include <Python.h>
static PyObject *pyoError;
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include "numpy/arrayobject.h"

#include <iostream>

#include "s2mMusculoSkeletalModel.h"
#include "s2mKalmanReconsMarkers.h"

#include "processInput.hpp"


static PyObject * pyoNew(PyObject *self, PyObject *args);
static PyObject * pyo_nQ(PyObject *self, PyObject *args);
static PyObject * pyo_nMarkers(PyObject *self, PyObject *args);
static PyObject * pyo_getMarkers(PyObject *self, PyObject *args);
static PyObject * pyo_kalmanFilterKinematicsReconstruction(PyObject * dummy, PyObject *args);

static PyObject * pyo_testDebug(PyObject *self, PyObject *args);

static PyMethodDef s2mMethods[] = {
    {"new",         pyoNew,         METH_VARARGS, "Create a new pyoModel."},
    {"nb_q",        pyo_nQ,         METH_VARARGS, "Get number of generalized coordinates in them model."},
    {"nb_markers",  pyo_nMarkers,   METH_VARARGS, "Get number of the marker in them model."},
    {"get_markers", pyo_getMarkers, METH_VARARGS, "Perform a direct kinematics, return marker positions."},
    // {"get_mesh",    pyo_getMesh,    METH_VARARGS, "Get vertex and triangle at Q"},
    {"kalman_kinematics_reconstruction", pyo_kalmanFilterKinematicsReconstruction,
                                    METH_VARARGS, "Perform a kinematic reconstruction using a kalman filter algorithm."},
    {"test_debug",  pyo_testDebug,  METH_VARARGS, "Dummy sand box for the developper to safely debug interface codes."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef pyorbdlModule = {
   PyModuleDef_HEAD_INIT,
   "pyorbdl",   /* name of module */
   NULL, /* module documentation, may be NULL */
   -1,       /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
   s2mMethods
};

PyMODINIT_FUNC PyInit_pyorbdl(void)
{
    PyObject *m;

    m = PyModule_Create(&pyorbdlModule);
    if (m == NULL)
        return NULL;

    pyoError = PyErr_NewException("pyorbdl.error", NULL, NULL);
    Py_INCREF(pyoError);
    PyModule_AddObject(m, "error", pyoError);

    return m;
}

// __PYORBDL_HPP__
#endif