#include "pyorbdl.hpp"

static PyObject * pyoNew(PyObject *self, PyObject *args)
{
    // Get the path name
    const char * path;
    if (!PyArg_ParseTuple(args, "s", &path))
        return NULL;

    // Construct a model
    s2mMusculoSkeletalModel * m = new s2mMusculoSkeletalModel(s2mRead::readModelFile(path));
    if (m == NULL) {
        PyErr_SetString(pyoError, "The file could not be opened");
        return NULL;
    }

    // Important for preventing from segfault internally using Numpy. I don't know exactly what is does though
    import_array();

    // Return the model
    return PyLong_FromLong(long(m));
}

static PyObject * pyo_nMarkers(PyObject *self, PyObject *args)
{
    // Get the inputs
    long address_m;
    if ( !PyArg_ParseTuple(args, "l", &address_m) )
        return NULL;
    s2mMusculoSkeletalModel * m = reinterpret_cast<s2mMusculoSkeletalModel *>(address_m);

    // Return the number of markers
    return PyLong_FromLong(m->nTags());
}

static PyObject * pyo_nQ(PyObject *self, PyObject *args)
{
    // Get the inputs
    long address_m;
    if ( !PyArg_ParseTuple(args, "l", &address_m) )
        return NULL;
    s2mMusculoSkeletalModel * m = reinterpret_cast<s2mMusculoSkeletalModel *>(address_m);

    // Return the number of generalized coordinates
    return PyLong_FromLong(m->nbQ());
}

static PyObject * pyo_getMarkers(PyObject *dummy, PyObject *args)
{
// Get the inputs

    long address_m;
    PyObject * Q_in;
    if ( !PyArg_ParseTuple(args, "lO", &address_m, &Q_in) )
        return NULL;

    s2mMusculoSkeletalModel * m = reinterpret_cast<s2mMusculoSkeletalModel *>(address_m);
    std::vector<s2mGenCoord> allQ;
    if (!getGenCoord(m, Q_in, allQ)){
        return NULL;
    }

    unsigned int nTags = m->nTags();
    int nFrames = allQ.size();
    double * p_doubleArray = new double [4 * nTags * nFrames];
    for (int f = 0; f < nFrames; ++f){ // for each frame
        // Perform direct kinematics for this frame
        s2mGenCoord Q = allQ[f];
        std::vector<s2mNodeBone> allT = m->Tags(*m, Q);

        // Dispatch data into output array
        for (unsigned int i = 0; i < nTags; ++i){ // For each marker
            for (int j = 0; j < 4; ++j){ // For each XYZ component
                if (j < 3)
                    p_doubleArray[nTags*nFrames*j+nFrames*i+f] = allT[i][j];
                else
                    p_doubleArray[nTags*nFrames*j+nFrames*i+f] = 1;
            }
        }
    }
    int nArraySize = 3; // Markers are always 3D (XYZ1 x nMarkers x nFrames)
    npy_intp * arraySizes = new npy_intp[nArraySize];
    arraySizes[0] = 4;
    arraySizes[1] = nTags;
    arraySizes[2] = nFrames;

    PyArrayObject * c = (PyArrayObject *)PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, p_doubleArray);
    delete[] arraySizes;

    // Give ownership to Python so it will free the memory when needed
    PyArray_ENABLEFLAGS(c, NPY_ARRAY_OWNDATA);

    return PyArray_Return(c);
}