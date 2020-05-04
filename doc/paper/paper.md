---
title: 'biorbd: A C++, Python and MATLAB library to analyze and simulate the human body'
tags:
  - Python
authors:
  - name: Benjamin Michaud
    orcid: 0000-0002-5031-1048
    affiliation: 1
  - name: Mickaël Begon
    orcid: 0000-0002-4107-9160
    affiliation: 1
affiliations:
 - name: École de Kinesiologie et de Sciences de l'Activité Physique, Université de Montréal
   index: 1
date: May 1st, 2020
bibliography: paper.bib
---

# Summary
Biomechanics is at the interface of several fields of science, such as robotics, mathematics, physics and human physiology.
Although this creates an environment where ideas can emerge, one can quickly be overwhelmed by the numerous types of data to analyze simultaneously. 
Indeed, it is not uncommun for biochanical data set to be composed of skin markers data, forces data, electromyographic data, inertial measurement units data, etc., which by nature are not straightforward to combine.
That said, their most obvious meeting point is their respective relation to the movement of the human body: some are its cause (e.g. muscle excitations) and some are its consequence (e.g. the skin markers displacement).
This is where `biorbd` steps in. 

`biorbd` a is feature-based development biomechanics library that targets to analyze the biochanical data in a comprehensive and accessible manner with the idea that the meeting point of all the biomechanical data are the movement of the body.
Therefore, `bio` stands for biomechanics and `rbd` stands for `rigid body dynamics`. 
Hence, for a given musculoskelettal model, it provides all the functions for inverse flow---that is from skin markers measurements up to muscle excitations---and direct flow---that is from the muscle excitations up to skin markers.

Biomechanics can be computationnaly expensive, and, depending on the goal, may required to be performed real-time computation. 
This is why the choice was made to write the core of `biorbd` in C++. 
Although C++ provides rapidity of computation, it drastically lacts of the flexibility given by higher level languages that is well apprecitated in the biomechanical community. 
In order to align with the community needs, Python and MATLAB binders are also provided with `biorbd`.
This allows `biorbd` to be elegantly incorporated in the usual workflow of researcher without compromising the speed of computations. 

As stated before, by nature, biomechanical data are multidimensional and almost always time dependent.
Another challenge related to that is the visualization of the data and results. 
In order to help the user to better comprehend the results of its simulations, the Python visualizer `BiorbdViz` is provided (CITE). 
This visualizer allows to move the avatar as desired or to load preexisting movements, to record videos, and for the model with muscles to look at the interaction between the movement and different muscular outputs, such as maximal force. 

# What does it do specifically
Biomechanical analyses are usually based on one of the two (or a mixture) of the following flows: inverse and direct. 
The former takes the results from a movement (e.g. the skin markers) and infer the cause of this particular movement.
Conversely, the latter assumes a command (e.g. muscle excitations) and computes the effect of this particular command.

## Inverse flow
The following tools are available for the inverse flow.

*Inverse kinematics*: Determine the kinematics of the system (generalized coordinates) from body measurements (skin markers or inertial measurement units). 
The algorithm implemented to perform this calculation is the Extended Kalman Filter. 
The main advantage of this algorithm is its capability to deal with missing data and its capability to merge different data type elegantly. 

*Inverse dynamics*: Determine the generalized force set ($\tau$) that produced a given kinematics (generalized accelerations). 
That is solving the following equation for $\tau$:
$$
\tau = M(q)\ddot{q} + N(q, \dot{q})
$$
where $q$, $\dot{q}$ and $\ddot{q}$ are the generalized coordinates, velocities and accelerations, respectively, $M(q)$ is the mass matrix and $N(q, \dot{q})$ is the bias effect. 
All the inverse dynamics algorithms implemented in `RBDL` are available.

*Static optimization*: Determine the muscle activations ($\alpha$) set that produced a given force set ($\tau$). 
In brief, using a non-linear optimization (Ipopt (CITE)), it minimizes the muscle activations *p*-norm (with $p=2$ most of the time) that matches the $\tau$. 
In equation, it reads as follow:
$$
\begin{aligned}
    & \underset{\alpha \in \mathbb{R}^m}{\text{minimize}}
    & & \left\lVert\alpha\right\rVert_p \\
    & \text{subject to}
    & & \tau_{mus_i}(\alpha ,q, \dot{q}) - \tau_{kin_i}(q, \dot{q}, \ddot{q}) = 0, &\; i=1,\ldots,n \\
    & & &  0 \leq \alpha_{t_j} \leq 1, &\; j=1,\ldots,m
\end{aligned}
$$
where $\tau_{mus_i}(\alpha ,q, \dot{q})$ is the generalized forces computed from the muscle activations ($\alpha$) and $\tau_{kin_i}(q, \dot{q}, \ddot{q})$ is the generalized forces computed from inverse dynamics.
Static optimization is not the sole way to infer the muscle activations from a given $\tau_{kin_i}$, but it is definitely the most used in the community. 

## Direct flow
The following tools are available for the direct flow.

*Muscle dynamics*: Determine the muscle activations derivative from the muscle excitations. 
The actual equation implemented depends on the muscle model used. 
In any case, its purpose is to model the calcium release in the muscle that will trigger the muscle contraction. 

*Muscular joint torque*: Determine the generalized forces ($\tau_{mus}$) from a muscle activations set ($\alpha$). 
To compute this, the muscle length jacobian ($J_{mus}(q)$) is constructed and multiplied by the muscle forces ($F_{mus}(q, \dot{q}, \alpha)$):
$$
\tau_{mus} = J_{mus}(q) F_{mus}(q, \dot{q}, \alpha)
$$

*Forward dynamics*: Determine the generalized accelerations ($\ddot{q}$) produced by a given generalized forces set ($\tau$). 
That is solving the following equation for $\ddot{q}$:
$$
\ddot{q} = M(q)^{-1}\tau - N(q, \dot{q})
$$
where $q$ and $\dot{q}$ are the generalized coordinates and velocities, respectively, $M(q)$ is the mass matrix and $N(q, \dot{q})$ is the bias effect. 
All the forward dynamics algorithms implemented in `RBDL` are available (including those with contact constraints).

*Forward kinematics*: Determine the model outputs (e.g. skin markers, inertial measurement units orientations) from a given generalized coordinates ($q$). 

# On what is it built on
`biorbd` takes advantage of several highly efficient backends, namely `RBDL`, `eigen` and `CasADi`. 
The first, and probably the most important, is the `RBDL` library by Martin Feliz (CITE) that implements the Featherstone's equations of spatial geometry (CITE). 
In brief, `RBDL` is an efficient library that provides all the computation needed by `biorbd` to model the interactions between the body segments. 
This library was previously used in the field of robotics (CITE, CITE).
`biorbd` extends the capacities of `RBDL` by giving more common biomechanics nomenclature, adding relevant computations and algorithms and by adding a muscle module that to analyze the interactions between muscles and rigid bodies.

`RBDL` uses `eigen` for its linear algebra backends. 
This library is a highly efficient linear algebra C++ library. 
Although this backend is useful to quickly compute most of the required computation, it lacks the capability to provides fast and accurate derivative --- apart from those analytically determined. 
Therefore, the algorithmic differentiation library `CasADi` was added as an alternative linear algebra backend.
This allows to have for free the derivative of almost all the function of `RBDL`. 
This is particularly useful when using `biorbd` in an optimization.
Speaking of which, the reader is welcomed to have a look at the optimal control module `BiorbdOptim` (CITE).

# What about preexisting solutions
`OpenSim` and `Anybody` are two state-of-the-art solutions that provides similar analysis flows.
`Anybody` is a closed and proprietary software.
For this software, from an open source point-of-view the main reason to create another similar library is therefore self-explanatory.
Conversely, `OpenSim` is open-source and very well established in the biomechanical community. 
There are some reasons that explain the need for `biorbd`.

First, the implementation of the dynamics is based on different philosophy.
`biorbd` is ultimately based on the spatial geometry described by Featherstone (CITE), while `OpenSim` 

Different philosophy 
the simbody is much more dense and hard to modify (use of CasADi)
don't put all eggs in the same basket



# Acknowledgements
A huge thanks to Ariane Dang for her patience and contribution on writting the tests for the library!

# References
