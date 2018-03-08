#ifndef __PROCESSINPUT_HPP__
#define __PROCESSINPUT_HPP__

// processInput must be included by a function which include <Python.h> and define pyoError

#include <string>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}



int getGenCoord(s2mMusculoSkeletalModel* m, PyObject* Q_in, std::vector<s2mGenCoord> &Q_out){
    unsigned int nQ = m->nbQ();

    // iterate over row, col and timeFrame
    PyObject * iter_row = PyObject_GetIter(Q_in);
    unsigned int idxRow = 0, idxCol, idxFrame;
    bool firstFrame = true;
    while (true){
        PyObject * row = PyIter_Next(iter_row);
        if (!row){
            break;
        }

        PyObject * iter_col = PyObject_GetIter(row);
        idxCol = 0;
        while (true){
            PyObject * col = PyIter_Next(iter_col);
            if (!col){
                break;
            }

            PyObject * iter_sheet = PyObject_GetIter(col);
            idxFrame = 0;
            while (true){
                PyObject * frame = PyIter_Next(iter_sheet);
                if (!frame){
                    firstFrame = false;
                    break;
                }
                if (firstFrame){
                    // This should be done before the first while.. But I don't know how to get dimensions...
                    Q_out.push_back(s2mGenCoord(nQ));
                }
                // Stockage data
                Q_out[idxFrame][idxRow] = PyFloat_AsDouble(frame);
                ++idxFrame;

                // Release reference
                Py_DECREF(frame);
            }
            ++idxCol;

            // Release references
            Py_DECREF(col);
            Py_DECREF(iter_sheet);
        }
        // Generalized coordinate should have exactly one column
        if (idxCol != 1){
            PyErr_SetString(pyoError, "GeneralizedCoordinate must have exactly 1 column");
            return 0;
        }
        ++idxRow;

        // Release references
        Py_DECREF(row);
        Py_DECREF(iter_col);
    }

    // Release reference
    Py_DECREF(iter_row);

    // Make sure nQ is equal to nQ necessary
    if (idxRow != nQ){
        PyErr_SetString(pyoError, std::string("The model requires " + patch::to_string(nQ) + " values for the generalized coordinates").c_str());
        return 0;
     }

    // Return
    return 1;
}

// __PROCESSINPUT_HPP__
#endif