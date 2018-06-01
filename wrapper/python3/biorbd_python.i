/* File : biorbd_python.i */
%{
#define SWIG_FILE_WITH_INIT
#include "s2mMusculoSkeletalModel.h"
%}

%include "numpy.i"
%init %{
    import_array();
%}


/*
// typemaps.i is a built-in swig interface that lets us map c++ types to other
// types in our language of choice. We'll use it to map Eigen matrices to
// Numpy arrays.
%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>


%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorVectorXd) std::vector<Eigen::VectorXd>;

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)



%apply (int* IN_ARRAY1, int DIM1) {(int* channels, int nChannels)};

%extend s2mGenCoord{
    // Extend the constructor of s2mGenCoord so Numpy can easily construct one
    PyObject * get_points(int* markers, int nMarkers)
    {
        std::vector<int> _markers;
        for (int i = 0; i < nMarkers; ++i)
            _markers.push_back(markers[i]);
            // Get the data
            
        size_t nMarkers(markers.size());
        const ezc3d::c3d& c3d = *self;
        const std::vector<int>& markers = _markers;
        const std::vector<ezc3d::DataNS::Frame>& frames = c3d.data().frames();
        size_t nFrames(frames.size());
        double * data = new double[4 * nMarkers * nFrames];
        for (int f = 0; f < nFrames; ++f){
            for (int m = 0; m < nMarkers; ++m){
                const ezc3d::DataNS::Points3dNS::Point& point(frames[f].points().point(markers[m]));
                data[nMarkers*nFrames*0+nFrames*m+f] = point.x();
                data[nMarkers*nFrames*1+nFrames*m+f] = point.y();
                data[nMarkers*nFrames*2+nFrames*m+f] = point.z();
                data[nMarkers*nFrames*3+nFrames*m+f] = 1;
            }
        }

        // Export them to Python Object
        int nArraySize = 3;
        npy_intp * arraySizes = new npy_intp[nArraySize];
        arraySizes[0] = 4;
        arraySizes[1] = nMarkers;
        arraySizes[2] = nFrames;
        PyArrayObject * c = (PyArrayObject *)PyArray_SimpleNewFromData(nArraySize,arraySizes,NPY_DOUBLE, data);
        delete[] arraySizes;

        // Give ownership to Python so it will free the memory when needed
        PyArray_ENABLEFLAGS(c, NPY_ARRAY_OWNDATA);

        return PyArray_Return(c);
    }

 }
*/
%include ../ezc3d.i



