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
Biomechanics is at the interface of several fields of science, such as robotics, physics and human physiology.
Although this encourages the emergence of innovative ideas, one can quickly be overwhelmed by the numerous types of data to analyze simultaneously. 
It is not uncommun for biochanical data set to be composed of skin markers (SM) data, forces data, electromyographic (EMG) data, inertial measurement units (IMU) data, etc., which by nature are not straightforward to combine.
That said, their most obvious meeting point is their respective relation to the movement of the human body: some are its cause (e.g. EMG) and some are its consequence (e.g. the SM displacement).
This is where `biorbd` steps in. 

`biorbd` a is *feature-based development* biomechanics library that targets to analyze the biochanical data in a comprehensive and accessible manner with the meeting point of the data being the movement of the body.
Therefore, `bio` stands for biomechanics and `rbd` stands for `rigid body dynamics`. 
For a given musculoskelettal model, it therefore provides all the functions for inverse flow---that is from SM measurements up to EMG---and direct flow---that is from EMG to SM.

Biomechanics can be computationnaly expensive, and may require real-time computation, which explains that the core of `biorbd` is written in C++. 
Although C++ provides fast computation, it lacks of the flexibility of higher level languages well apprecitated by biomechanical community. 
In order to align with the community needs, Python and MATLAB binders are provided with `biorbd`.
This allows `biorbd` to be elegantly incorporated in the usual workflow of researchers without compromising the speed of computations. 

Finally, biomechanical data are often multidimensional and almost always time dependent which challenges the visualization of the results. 
The Python visualizer `BiorbdViz` (CITE) can be used to help with that. 
This visualizer allows to move the model by hand or by results, to record videos, and, if the model includes muscles, to vizualize the interactions between the movement and muscular outputs (e.g. maximal force). 

# Inverse and direct flow overlook
Biomechanical analyses are usually based on one of the two (or a mixture) of the following flows: inverse and direct. 
The former takes measurement from a movement (e.g. the SM) and infers its cause.
Conversely, the latter assumes a command (e.g. EMG) and estimates the resulting movement.

## Inverse flow
*Inverse kinematics*: Estimates the generalized coordinates ($q$)---that is the kinematics---from body measurements (e.g. SM, IMU, etc.). 
The algorithm implemented is the Extended Kalman Filter, which has the main advantage to elegantly dealing with missing data and to merging multiple data source.

*Inverse dynamics*: Estimates the generalized forces ($\tau$) that produced a given generalized accelerations ($\ddot{q}$) obtained from the second deritave of $q$. 
That is solving the following equation for $\tau$:
$$
\tau = M(q)\ddot{q} + N(q, \dot{q})
$$
where $\dot{q}$ is the generalized velocities, $M(q)$ is the mass matrix and $N(q, \dot{q})$ are the bias effect. 

*Static optimization*: Estimates the muscle activations ($\alpha$) that produced a given $\tau$. 
In brief, using a non-linear optimization (Ipopt (CITE)), it minimizes the muscle activations *p*-norm (with $p$ usually being $2$) that matches a give $\tau$. 
$$
\begin{aligned}
    & \underset{\alpha \in \mathbb{R}^m}{\text{minimize}}
    & & \left\lVert\alpha\right\rVert_p \\
    & \text{subject to}
    & & \tau_{mus_i}(\alpha ,q, \dot{q}) - \tau_{kin_i}(q, \dot{q}, \ddot{q}) = 0, &\; i=1,\ldots,n \\
    & & &  0 \leq \alpha_{t_j} \leq 1, &\; j=1,\ldots,m
\end{aligned}
$$
where $\tau_{mus_i}(\alpha ,q, \dot{q})$ is the $\tau$ computed from the $\alpha$ and $\tau_{kin_i}(q, \dot{q}, \ddot{q})$ being the output of the inverse dynamics.
Static optimization is not the sole way to infer the muscle activations from a given $\tau_{kin_i}$, but it is definitely the most used in the community. 

## Direct flow
*Muscle acitvation dynamics*: Estimates the muscle activations derivative ($\dot{\alpha}$) from the muscle excitations. 
It models the calcium release in the muscle that triggers the muscle contraction. 
Multiple activation dynamics are implemented (e.g. (CITE), Buchanan (CITE)).

*Muscular joint torque*: Estimates the muscles generalized forces ($\tau_{mus}$) from muscle activations ($\alpha$). 
To compute this, the muscle length jacobian ($J_{mus}(q)$) is constructed and multiplied by the muscle forces ($F_{mus}(q, \dot{q}, \alpha)$):
$$
\tau_{mus} = J_{mus}(q)^T F_{mus}(q, \dot{q}, \alpha)
$$

*Forward dynamics*: Estimates the generalized accelerations ($\ddot{q}$) from a given $\tau$. 
That is solving the following equation for $\ddot{q}$:
$$
\ddot{q} = M(q)^{-1}\tau - N(q, \dot{q})
$$
All the forward dynamics implemented in `RBDL` are available.

*Forward kinematics*: Estimates the model outputs (e.g. SM, IMU, etc.) from a given $q$. 

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
`OpenSim` and `Anybody` are two state-of-the-art softwares that provide similar analysis flows.
`Anybody` is a closed and proprietary software.
For this one, from an open source point-of-view, the main reason to create another similar library is therefore self-explanatory.
Conversely, `OpenSim` is open-source and very well established in the biomechanical community. 
There are two main reasons that explain the need for `biorbd`.

First, `biorbd` is made to be more lightweight and flexible than `OpenSim`. 
The target audience of `OpenSim` are those who wants to analyze movements from the GUI or by using macros to call the API. 
Great care is therefore taken to the frontend API. 
The backend is however more hermetic as it targets efficiency more than flexibility, at least from the point of view of a new programmer. 
The use of the multibody physics of `Simbody` adds a level of complexity since this library is generic enough to be used for "internal coordinate and coarse grained molecule modeling, large scale mechanical models like skeletons, and anything else that can be modeled as bodies interconnected by joints, acted upon by forces, and restricted by constraints." (CITE https://simtk.org/projects/simbody/). 
Implementing an automatic differentiation backend as it was done for `biorbd` is therefore harder to do and has a huge impact on a lot of other software that simbody depends on. 
It is for instance unlikely that such a fundamental modification would be integrated in the stable branch of `Simbody` any time soon. 
`biorbd` is more straightforward, especially by being less compact---by using explicit code instead of templating---and by being less generic---that is being limitted to musculoskettal modelling.
Although it has its drawbacks, this type of code is much easier to modify and test by the community. 

A second reason is as a community, it is important to implement similar but slightly different tools so they can be cross-valided. 
Two papers (CITE, CITE) recently compared the outputs of `Anybody` and `OpenSim` for different functions, and came to the generic conclusion that they were different.
Due to the closed source nature of `Anybody`, direct comparison between the actual codes that produced these differences is impossible.
Although, the authors provide plausible explanations for these differences, they have to assume that the implementation of the algorithms are without mistakes for both of the softwares. 
Having multiple open source softwares that produce similar ends by different means, as `biorbd` and `OpenSim` do, is an insurance for high quality for the users: "do not put all eggs in the same basket", they say. 
Therefore, in our view, `biorbd` doesn't compete with `OpenSim` more than they complete each others. 

# Acknowledgements
A huge thanks to Ariane Dang for her patience and contribution on writting the tests for the library!

# References
