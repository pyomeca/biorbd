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
For a given musculoskelettal model, it implements all the functions for inverse---that is from skin markers measurements up to muscle excitations---and direct flows---that is from the muscle excitations up to skin markers.

Biomechanics can be computationnaly expensive and, depending on the goal, may required to be performed real-time. 
This is why the choice was made to write the `biorbd` core in C++. 
Although C++ provides rapidity of computation, it drastically lacts of the flexibility given by higher level languages.
Because of that, C++ is not so much used in the biomechanical community. 
In order to align with the community needs, Python and MATLAB binders are also provided.
This allows `biorbd` to be elegantly incorporated in the usual workflow of researcher without compromising the speed of computations. 

By nature, biomechanical data are multidimensional and time dependent.
Because of that visualizing the results may be a challenge. 
In order to help the user to comprehend the results of its simulations, the Python visualizer `BiorbdViz` is provided. 
This visualizer allows to move the avatar as desired or to load preexisting movements, to record videos, and for the model with muscles to look at the effect of the movement on different muscular outputs, such as maximal force. 

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

# What does it do specifically
Image? 

Inverse flow

Direct

# What about preexisting solutions
`OpenSim` and `Anybody`


# Acknowledgements
A huge thanks to Ariane Dang for her patience and contribution on writting the tests for the library!

# References
