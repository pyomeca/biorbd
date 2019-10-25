# BIORBD

BIORBD is a library to analyze biomechanical data. It provides several useful functions for the direct and inverse flow including rigid body (based on *Feathestone* equations implemented in RBDL) and muscle elements.

Biomechanical data are often analyzed using similar flow, that is inverse or direct. BIORBD implements these common analyses providing high-level and easy to use Python and MATLAB interfaces of an efficient C++ implementation. 

So, without further ado, let's begin our investigation of BIORBD!

# How to install
There are two main ways to install BIORBD on your computer: installing the binaries from Anaconda (easiest, but limited to C++ and Python3) or compiling the source code yourself (more versatile and up to date; for C++, Python3 and MATLAB).

## Anaconda (For Windows, Linux and Mac)
The easiest way to install BIORBD is to download the binaries from Anaconda (https://anaconda.org/) repositories (binaries are not available though for MATLAB). The project is hosted on the conda-forge channel (https://anaconda.org/conda-forge/biorbd).

After having installed properly an anaconda client [my suggestion would be Miniconda (https://conda.io/miniconda.html)] and loaded the desired environment to install BIORBD in, just type the following command:
```bash
conda install -c conda-forge biorbd
```
The binaries and includes of the core of BIORBD will be installed in `bin` and `include` folders of the environment respectively. Moreover, the Python3 binder will also be installed in the environment.

The current building status for Anaconda release is as follow.

| Name | Downloads | Version | Platforms |
| --- | --- | --- | --- |
| [![Conda Recipe](https://img.shields.io/badge/recipe-biorbd-green.svg)](https://anaconda.org/conda-forge/biorbd) | [![Conda Downloads](https://img.shields.io/conda/dn/conda-forge/biorbd.svg)](https://anaconda.org/conda-forge/biorbd) | [![Conda Version](https://img.shields.io/conda/vn/conda-forge/biorbd.svg)](https://anaconda.org/conda-forge/biorbd) | [![Conda Platforms](https://img.shields.io/conda/pn/conda-forge/biorbd.svg)](https://anaconda.org/conda-forge/biorbd) |

## Compiling (For Windows, Linux and Mac)
The main drawback with downloading the pre-compiled version from Anaconda is that this version may be out-of-date (even if I do my best to keep the release versions up-to-date). Moreover, since it is already compiled, it doesn't allow you to modify BIORBD if you need to. Therefore, a more versatile way to enjoy BIORBD is to compile it by yourself.

The building status for the current BIORBD branches is as follow

| Name | Status |
| --- | --- |
| master | [![Build Status](https://travis-ci.org/pyomeca/biorbd.svg?branch=master)](https://travis-ci.org/pyomeca/biorbd) |

### Dependencies
BIORBD relies on several libraries (namely Boost (https://www.boost.org/), eigen (http://eigen.tuxfamily.org), dlib (http://dlib.net/), rbdl (https://rbdl.bitbucket.io/) and Ipopt (https://github.com/coin-or/Ipopt)) that one must install prior to compiling. Fortunately, all these dependencies are also hosted on the *conda-forge* channel of Anaconda. Therefore the following command will install everything you need to compile BIORBD:
```bash
conda install -c conda-forge boost eigen dlib-cpp rbdl ipopt
```
Please note that ```ipopt``` is optional, but is required for the *Static optimization* module. 

Additionnally, for the Python3 interface requires *numpy* (https://numpy.org/) and *SWIG* (http://www.swig.org/). Again, one can easily install these dependencies from Anaconda using the following command:
```bash
conda install -c conda-forge numpy swig
```

Finally, the MATLAB interface (indeed) requires MATLAB to be installed.

If ones is interested in developping BIORBD, the ```googletest``` suite is required to test your modifications. Fortunately, the CMake should download and compile the test suite for you!

### CMake
BIORBD comes with a CMake (https://cmake.org/) project. If you don't know how to use CMake, you will find many examples on Internet. The main variables to set are:

> `CMAKE_INSTALL_PREFIX` Which is the `path/to/install` BIORBD in. If you compile the Python3 binder, a valid installation of Python with Numpy should be installed relatived to this path.
>
> `BUILD_SHARED_LIBS` If you wan to build BIORBD in a shared `TRUE` or static `FALSE` library manner. Default is `TRUE`. Please note that due to the dependencies, on Windows BIORBD must be statically built.
>
> `CMAKE_BUILD_TYPE` Which type of build you want. Options are `Debug`, `RelWithDebInfo`, `MinSizeRel` or `Release`. This is relevant only for the build done using the `make` command. Please note that you will experience a slow BIORBD library if you compile it without any optimization (i.e. `Debug`), especially for all functions that requires linear algebra. 
>
> `BUILD_EXAMPLE` If you want (`TRUE`) or not (`FALSE`) to build the C++ example. Default is `TRUE`.
>
> `BUILD_TESTS` If you want (`ON`) or not (`OFF`) to build the tests of the project. Please note that this will automatically download gtest (https://github.com/google/googletest). Default is `OFF`.
>
> `BUILD_DOC` If you want (`ON`) or not (`OFF`) to build the documentation of the project. Default is `OFF`.
>
> `BINDER_PYTHON3` If you want (`ON`) or not (`OFF`) to build the Python binder. Default is `OFF`.
>
> `Python3_EXECUTABLE`  If `BINDER_PYTHON3` is set to `ON` then this variable should point to the Python executable. This python should have *SWIG* and *Numpy* installed with it. This variable should be found automatically, but Anaconda finds the base prior to the actual environment, so one should gives attention to that particular variable.
>
> `SWIG_EXECUTABLE`  If `BINDER_PYTHON3` is set to `ON` then this variable should point to the SWIG executable. This variable will be found automatically if `Python3_EXECUTABLE` is properly set.
>
> `BINDER_MATLAB` If you want (`ON`) or not (`OFF`) to build the MATLAB binder. Default is `OFF`.
>
> `MATLAB_ROOT_DIR` If `BINDER_MATLAB` is set to `ON` then this variable should point to the root path of MATLAB directory. Please note that the MATLAB binder is based on MATLAB R2018a API and won't compile on earlier versions. This variable should be found automatically, except on Mac where the value should manually be set to the MATLAB in the App folder.
>
> `MATLAB_biorbd_INSTALL_DIR` If `BINDER_MATLAB` is set to `ON` then this variable should point to the path where you want to install BIORBD. Typically, this is `{MY DOCUMENTS}/MATLAB`. The default value is the toolbox folder of MATLAB. Please note that if you leave the default value, you will probably need to grant administrator rights to the installer. 

# How to use
BIORBD provides as much as possible explicit names for the filter so one can intuitively find what he wants from the library. Still, this is a C++ library and it can be sometimes hard to find what you need. Due to the varity of functions implemented in the library, minimal examples are shown here. One is encourage to have a look at the `example` and `test` folders to get a better overview of the possibility of the API. For an in-depth detail of the API, the Doxygen documentation (to come) is the way to go.

## The C++ API
The core code is written in C++, meaning that you can fully use BIORBD from C++.  Moreover, the linear algebra is using the Eigen library which makes it fairly easy to perform further computation and analyses.
The informations that follows is a basic guide that should allow you to perform everything you want to do.

### Create an empty yet valid model
To create a new valid yet empty model, just call the `biorbd::Model` class without parameter.
```C++
#include "biorbd.h"
int main()
{
    biorbd::Model myModel;
}
```
This model can thereafter be populated using the *biorbd* add methods. Even if this is not the prefered way of loading a model, one can have a look at the *src/ModelReader.cpp* in order to know what functions that must be called to populate the model manually. 

### Read and write a bioMod file
The prefered method to load a model is to read the in-house *.bioMod* format file.  To do so, one must simply call the `biorbd::Model` constructur with a valid path to the model. Afterward, one can modify manually the model and write it back to a new file. 
```C++
#include "biorbd.h"
int main()
{
    biorbd::Model myModel myModel("path/to/mymodel.bioMod");
    // Do some changes...
    biorbd::Writer::writeModel(myModel, "path/to/newFile.bioMod");
    return 0;
}
```
Please note that on Windows, the path must be `/` or `\\` separated (and not only`\`), for obvious reasons. 

### Perform some analyses
BIORBD is made to work with the RBDL functions (the doc can be found here https://rbdl.bitbucket.io/). Therefore, every functions available in RBDL is also available on BIORBD. Additionnal are of course also made available, for example the whole muscle module. 

The most obvious and probably the most used function is the forward kinematics, where one knows the configuration of the body and is interested in the resulting position of skin markers. The following code performs that task.
```C++
#include "biorbd.h"
int main()
{
    // Load the model
    biorbd::Model model("path/to/model.bioMod");
    
    // Prepare the model
    biorbd::rigidbody::GeneralizedCoordinates Q(model); 
    Q.setOnes()/10; // Set the model position
    
    // Perform forward kinematics
    std::vector<biorbd::rigidbody::NodeBone> markers(model.markers(Q));
    
    // Print the results
    for (auto marker : markers)
        std::cout << marker.name() << " is at the coordinates: " << marker.transpose() << std::endl;
    return 0;
}
```

Another common analysis to perform is to compute the effect of the muscles on the acceleration of the model. Assuming that the model that is loaded has muscles, the following code perform this task.
```C++
#include "biorbd.h"
int main()
{
    // Load the model
    biorbd::Model model("path/to/model.bioMod");
    
    // Prepare the model
    biorbd::rigidbody::GeneralizedCoordinates Q(model), Qdot(model); // position, velocity
    Q.setOnes()/10; // Set the model position
    Qdot.setOnes()/10; // Set the model velocity
    // Muscles activations
    std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> states(model.nbMuscleTotal());
    for (auto& state : states){
        state = std::make_shared<biorbd::muscles::StateDynamics>();
        state->setActivation(0.5); // Set the muscle activation
    }

    // Compute the joint torques based on muscle
    biorbd::rigidbody::GeneralizedTorque muscleTorque(
                model.muscularJointTorque(states, true, &Q, &Qdot));

    // Compute the acceleration of the model due to these torques
    biorbd::rigidbody::GeneralizedCoordinates Qddot(model);
    RigidBodyDynamics::ForwardDynamics(model, Q, Qdot, muscleTorque, Qddot);

    // Print the results
    std::cout << " The joints accelerations are: " << Qddot.transpose() << std::endl;
    return 0;
}
```

There are many other analyses and filters that are available. Please refer to the BIORBD and RBDL Docs to see what is available. 

## MATLAB
MATLAB (https://www.mathworks.com/) is a prototyping langage largely used in industry and fairly used by the biomechanical scientific community. Despite the existence of Octave as an open-source and very similar language or the growing popularity of Python as a free and open-source alternative, MATLAB remains an important player as a programming languages. Therefore BIORBD comes with a binder for MATLAB (that can theoretically used with Octave as well with some minor changes to the CMakeLists.txt file).

Most of the functions available in C++ are also available in MATLAB. Still, they were manually binded, therefore it may happen that some important one (for you) are not there. If so, do not hesitate to open an issue on GitHub to required the add of that particular function. The philosophy behind the MATLAB binder is that you open a particular model and a reference to that model is gave back to you. Thereafter, the functions can be called, assuming the pass back that model reference. That implies, however, that ones must himself deallocate the memory of the model when it is no more needed. Failing to do so results in an certain memory leak.

### Perform some analyses
Please find here the same tasks previously described for the C++ interface done in the MATLAB interface. Notice that the MATLAB interface takes advantage of the matrix nature of MATLAB and therefore can usually perform the analyses on multiple frames at once. 

Forward kinematics can be performed as follow
```MATLAB
nFrames = 10; % Set the number of frames to simulate

% Load the model
model = biorbd('new', 'path/to/model.bioMod');

% Prepare the model
Q = ones(biorbd('nQ', model), nFrames)/10; % Set the model position

% Perform the forward kinematics
markers = biorbd('markers', model, Q);

% Print the results
disp(markers);

% Deallocate the model
biorbd('delete', model);
```

The joint accelerations from muscle activations can be performed as follow
```MATLAB
nFrames = 10; % Set the number of frames to simulate

% Load the model
model = biorbd('new', 'path/to/model.bioMod');

% Prepare the model
Q = ones(biorbd('nQ', model), nFrames)/10; % Set the model position
Qdot = ones(biorbd('nQdot', model), nFrames)/10; % Set the model velocity
activations = ones(biorbd('nMuscles', model), nFrames)/2; % Set muscles activations

% Compute the joint torques based on muscle
jointTorque = biorbd('jointTorqueFromActivation', model, activations, Q, Qdot);

% Compute the acceleration of the model due to these torques
Qddot = biorbd('forwardDynamics', model, Q, Qdot, jointTorque);

% Print the results
disp(Qddot);

% Deallocate the model
biorbd('delete', model);
```

### Help
One can print all the available functions by type the `help` command
```MATLAB
biorbd('help')
```
Please note that it seems that on Windows, the command returns nothing. One must therefore look in the source code (`biorbd/binding/matlab/Matlab_help.h`) what should the command have returned.


## Python 3
Python (https://www.python.org/) is a scripting language that has taken more and more importance over the past years. So much that now it is one of the preferred language of the scientific community. Its simplicity yet its large power to perform a large variety of tasks makes it a certainty that its popularity won't decrease for the next years.

To interface the C++ code with Python, SWIG is a great tool. It creates very rapidly an interface in the target language with minimal code to write. However, the resulting code in the target language can be far from being easy to use. In effect, it gives a mixed-API not far from the original C++ language, which may not comply to best practices of the target language. When this is useful to rapidly create an interface, it sometime lacks of user-friendliness and expose the user to the possibility of the C++ such as segmentation fault (unlike the MATLAB API which won't suffer from this devil problem). 

BIORBD interfaces the C++ code using SWIG. While it has some inherent limit as discussed previously, it has the great advantage of providing almost for free the complete API. Because of that, much more of the C++ API is interfaced in Python than the MATLAB one. Again, if for some reason, part of the code which is not accessible yet is important for you, don't hesitate to open an issue asking for that particular feature!

### Perform some analyses
Please find here the same tasks previously described for the C++ interface done in the Python3 interface. Please note that the interface usually takes advantage of the numpy arrays in order to interact with the user while a vector is needed. 

Forward kinematics can be performed as follow
```Python
import numpy as np
import biorbd

# Load the model
model = biorbd.Model('path/to/model.bioMod')

# Prepare the model
Q = np.ones(model.nbQ())/10  # Set the model position

# Perform the forward kinematics
markers = model.markers(Q)

# Print the results
for marker in markers:
    print(marker.get_array())

```

The joint accelerations from muscle activations can be performed as follow
```Python
import numpy as np
import biorbd

# Load the model
model = biorbd.Model('path/to/model.bioMod')

# Prepare the model
Q = np.ones(model.nbQ())/10  # Set the model position
Qdot = np.ones(model.nbQ())/10  # Set the model velocity
states = biorbd.VecBiorbdMuscleStateDynamics(model.nbMuscleTotal())
for state in states:
    state.setActivation(0.5)  # Set muscles activations

# Compute the joint torques based on muscle
joint_torque = model.muscularJointTorque(states, Q, Qdot)

# Compute the acceleration of the model due to these torques
Qddot = model.ForwardDynamics(Q, Qdot, joint_torque)

# Print the results
print(Qddot.get_array())

```
# Model files
## *bioMod* files
The preferred method to load a model is by using a *.bioMod* file. This type of file is an in-house language that describes the segments of the model, their interactions and additionnal elements attached to them. The following section describe the structure of the file and all the tags that exists so far. 

Comments can be added to the file in a C-style way, meaning that everything a on line following a `//` will be considered as a comment and everything between `/*` and `*/` will also be ignored. 

Please note that the *bioMod* is not case dependent, so `Version`and `version` are for instance fully equivalent. The *bioMod* reader also ignore the tabulation, which is therefore only aesthetic. 

When a tag waits for multiple values, they must be separate by a space, a tabulation or a return of line. Also, anytime a tag waits for a value, it is possible to use simple equations (assuming no spaces are used) and/or variables. For example, the following snippet is a valid way to set the gravity parameter to $(0, 0, -9.81)$. 
```c
variables
    $my_useless_variable 0
endvariables
gravity 2*(1-1) -2*$my_useless_variable
        -9.81
```

### Header
#### version
The very first tag that **must** appear at the first line in file is the version of the file. The current version of the *.bioMod* files is $4$. Please note that most of the version are backward compatible, unless specified. This tag waits for $1$ value.
```c
version 4
```
From that point, the order of the tags is not important, header can even be at the end of the file. For simplicity though we suggest to put everything related to the header at the top of the file. 

#### gravity
The `gravity` tag is used to reorient and/or change the magnitude of the gravity. The default value is $(0, 0, -9.81)$. This tag waits for $3$ values.
```c
// Restate the default value
gravity 0 0 -9.81
```

#### root_actuated
The `root_actuated` tag allows to change the number of non-zero degrees-of-freedom on the first segment. Typically, if the whole body is modeled, this is set to false, otherwise to true. Please note, that even if this tag is false, the *GeneralizedTorque* vector sent to `RigidBodyDynamic` must be of the same size as *qddot*. The default value is true ($1$). This tags waits for $1$ value. 
```c
// Restate the default value
root_actuated 1
```

#### external_forces
The `external_forces` tag allows to inform BIORBD that external forces exists in the model. So far, this tag has no effect. The default value is false ($0$) This tags waits for $1$ value. 
```c
// Restate the default value
external_forces 0
```

#### variables / endvariables
The `variables / endvariables` tag pair allows to declare variables that can be used within the file. This allows for example to template the *bioMod* file by only changing the values in the variables. Please note that contrary to the rest of the file, the actual variables are case dependent. 

The `\$` sign is mandatory and later in the file, everything with that sign followed by the same name will be converted to the values specified in the tag. 
```c
// Restate the default value
variables
    $my_first_variable_is_an_int 10
    $my_second_variable_is_a_double 10.1
    $myThirdVariableIsCamelCase 1
    $myLastVariableIsPi pi
endvariables
```
As you may have noticed, the constant $\pi$ is defined as $3.141592653589793$.

### Definition of the model
A BIORBD model consists of a chain of segment, linked by joints with up to six DoF (3 translations, 3 rotations). It is imperative when attaching something to a segment of the model that particular segment must have been previously defined. For instance, if the `thorax` is attached to the `pelvis`, then the latter must be defined before the former in the file. 

#### segment
The `segment xxx / endsegment`tag pair is the core of a *bioMod* file. It describes a segment of the model with the name `xxx`, that is most of the time a bone of the skeleton. For internal reasons, the name cannot be `root`. The `xxx` must be present and consists of $1$ string. The segment is composed of multiple subtags, described here. 

##### parent
The `parent` tag is the name of the segment that particular segment is attached to. If no segment parent is provided, it is considered to be attached to the environment. The parent must be defined earlier in the file and is case dependent. This tag waits for $1$ string.

#### translations
The `translations` tag specifies the number of degrees-of-freedom in translation and their order. The possible values are `x`, `y` and/or `z` combined whatever fits the model. Please note that the vector of generalized coordinate will comply the the order wrote in this tag. If no translations are provided, then the segment has no translation relative to its parent. This tag waits for $1$ string.

#### rotations
The `rotations` tag specifies the number of degrees-of-freedom in rotation and their order. The possible values are `x`, `y` and/or `z` combined whatever fits the model. Please note that the vector of generalized coordinate will comply the the order wrote in this tag. If no rotations are provided, then the segment has no rotation relative to its parent. This tag waits for $1$ string.

#### mass
The `mass` tag specifies the mass of the segment in kilogram. This tag waits for $1$ value.

#### inertia
The `inertia` tag allows to specify the matrix of inertia of the segment. It waits for $9$ values.



# How to contribute

You are very welcome to contribute to the project! There are to main ways to contribute. 

The first way is to actually code new features for BIORBD. The easiest way to do so is to fork the project, make the modifications and then open a pull request to the main project. Don't forget to add your name to the contributor in the documentation of the page if you do so!

The second way is to open issues to report bugs or to ask for new features. I am trying to be as reactive as possible, so don't hesitate to do so!

# Graphical User Interface (GUI)
For now, there is no GUI for the C++ interface and the MATLAB one is so poor I decided not to release it. However, there is a Python interface that worths to have a look at. Installation procedure and documentation can be found at the GitHub repository (https://github.com/pyomeca/biorbd-viz).

# Documentation
The documentation is not ready yet, but will come soon, when it is ready the next paragraph applies.

The documentation is automatically generated using Doxygen (http://www.doxygen.org/). You can compile it yourself if you want (by setting `BUILD_DOC` to `ON`). Otherwise, you can access a copy of it that I try to keep up-to-date in the Documentation project of pyomeca (https://pyomeca.github.io/Documentation/) by selecting `biorbd`. 

# Troubleshoots
Despite my efforts to make a bug-free library, BIORBD may fails sometimes. If it does, please refer to the section below to know what to do. I will fill this section with the issue over time.

## Slow BIORBD
If you experience a slow BIORBD, you are probably using a non optimized version, that is compiled with `debug` level. Please use at least `RelWithDebInfo` level of optimization while compiling BIORBD. 

If you actually are using a released level of optimization, you may actually experiencing a bug. You are therefore welcomed to provide me with a minimal example of your slow code and I'll see how to improve the speed!

# Cite
If you use BIORBD, we would be grateful if you could cite it as follows:

```
@misc{Michaud2018biorbd,
    author = {Michaud, Benjamin and Begon, Mickael},
    title = {BIORBD: Toolbox for biomechanical analyses},
    howpublished={Web page},
    url = {https://github.com/pyomeca/biorbd},
    year = {2018}
}
```
