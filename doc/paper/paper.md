---
title: 'biorbd: A C++, Python and MATLAB library to analyze and simulate the human body biomechanics'
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
Biomechanics is at the interface of several fields of science, such as robotics, mechanics and human physiology.
Although this transdisciplinarity encourages new ideas to emerge, the numerous data types to analyze simultaneously can be overwhelming. 
It is not uncommun for biomechanical data set to be composed of skin markers trajectories (SMT), contact forces, electromyographic (EMG) signal, inertial measurement units (IMU) kinematics, etc., which by nature are not straightforward to combine.
This is at their meeting point---the body movement---that `biorbd` steps in: `bio` standing for biomechanics and `rbd` for `rigid body dynamics`. 
`biorbd` is a *feature-based development* library that targets to manipulate biomechanical data in a comprehensive and accessible manner.
For a given musculoskeletal model, it provides functions for inverse flow---that is from SMT to EMG---and direct flow---that is from EMG to SMT.

Biomechanics often requires computationally expensive or real-time computations.
This is why the core of `biorbd` is written in C++. 
Although this language provides fast computations, it lacks the flexibility of higher-level languages. 
To meet the needs of the biomechanics community, Python and MATLAB binders are provided with `biorbd`.
This allows `biorbd` to be elegantly incorporated in the usual workflows of researchers without compromising the required speed. 

Finally, biomechanical data are often multidimensional and almost always time-dependent which can be challenging to visualize. 
`BiorbdViz` (CITE), a Python visualizer, can be used to help. 
This visualizer allows to animate the model, record videos, and, for model that includes muscles, plot muscular outputs against various features of the movement. 

# A `biorbd` overview, the inverse and direct flow
Biomechanical analyses are usually based on one (or a mixture) of the inverse or direct flow (CITE). 
In brief, the former uses measurements from a movement (e.g., the SMT) and infers its cause, while the latter assumes a command (e.g., EMG) and outputs the resulting kinematics.

## Inverse flow
*Inverse kinematics*: Estimates the generalized coordinates ($q$)—that is the body kinematics—from body measurements (e.g., SMT, IMU, etc.). 
The main algorithm implemented is the Extended Kalman Filter which has the main advantage of elegantly merging multiple data sources and to naturally taking care of missing data.

*Inverse dynamics*: Estimates the generalized forces ($\tau$) that produced a given generalized acceleration ($\ddot{q}$) (the second derivative of $q$):
$$
\tau = M(q)\ddot{q} + N(q, \dot{q})
$$
where $\dot{q}$ is the generalized velocities, $M(q)$ is the mass matrix and $N(q, \dot{q})$ are the bias effect. 

*Static optimization*: Estimates the muscle activation ($\alpha$) that produced a given $\tau$. 
In brief, using a non-linear optimization (Ipopt (CITE)), it minimizes the muscle activation *p*-norm (with $p$ usually being $2$) that matches a given $\tau$. 
$$
\begin{aligned}
    & \underset{\alpha \in \mathbb{R}^m}{\text{minimize}}
    & & \left\lVert\alpha\right\rVert_p \\
    & \text{subject to}
    & & \tau_{mus_i}(\alpha ,q, \dot{q}) - \tau_{kin_i}(q, \dot{q}, \ddot{q}) = 0, &\; i=1,\ldots,n \\
    & & &  0 \leq \alpha_{t_j} \leq 1, &\; j=1,\ldots,m
\end{aligned}
$$
where $\tau_{mus_i}(\alpha ,q, \dot{q})$ and $\tau_{kin_i}(q, \dot{q}, \ddot{q})$ are $\tau$ computed from muscle forces ($F_{mus}(\alpha, q, \dot{q})$) and inverse dynamics, respectively.

## Direct flow
*Muscle activation dynamics*: Estimates the muscle activation derivative ($\dot{\alpha}$) from the muscle excitation---that is the calcium release in the muscle that triggers the muscle contraction. 
Multiple activation dynamics are implemented (e.g. (CITE), Buchanan (CITE)).

*Muscular joint torque*: Estimates the $\tau_{mus}$ from muscle forces ($F_{mus}(q, \dot{q}, \alpha)$) estimated from $\alpha$ using a muscle model (e.g., Thelen CITE):
$$
\tau_{mus} = J_{mus}(q)^T F_{mus}(q, \dot{q}, \alpha)
$$
where $J_{mus}(q)$ is the muscle length Jacobian. 

*Forward dynamics*: Estimates the $\ddot{q}$ from a given $\tau$:
$$
\ddot{q} = M(q)^{-1}\tau - N(q, \dot{q})
$$
All the forward dynamics implemented in `RBDL` (CITE) are available.

*Forward kinematics*: Estimates the model kinematics outputs (e.g., SMT, IMU) from a given $\ddot{q}$ time integrated twice. 

# The dependencies
`biorbd` takes advantage of efficient back end, especially `RBDL` and `CasADi`. 
`RBDL` library, written by Martin Feliz (CITE), implements the Featherstone's equations of spatial geometry (CITE), successfully previously used in the field of robotics (CITE, CITE)
That library provides the core algorithms for body dynamics. 
`biorbd` extends `RBDL` by giving common biomechanics nomenclature, and adding a muscle module, amongst others.
`RBDL` is based on the highly efficient C++ linear algebra library `Eigen` (CITE). 

Although `Eigen` is flexible and fast enough for most of the common usage, it lacks the capability to provide derivatives of functions too complex to be analytically determined. 
Therefore, `RBDL` was also augmented with the algorithmic differentiation library `CasADi`.
This allows to precompute at low cost the derivatives of almost all the function in `RBDL`. 
This is particularly useful when using `biorbd` in a gradient-based optimization setting.

# The already-existing software in biomechanics
`OpenSim` and `Anybody` are state-of-the-art software that provides similar analysis flows.
`Anybody` is a closed and proprietary software, which self-explains for an open-source community the reasons to create another library.
Conversely, `OpenSim` is open-source and very well established in the biomechanical community. 
There are two main reasons that explain the need for `biorbd`.

First, `biorbd` is made to be more lightweight and flexible than `OpenSim`. 
The target audience of `OpenSim` is mostly GUI and macro-based users. 
The front end is great, but the back end is denser as it targets efficiency more than flexibility. 
The use of the multibody physics of `Simbody` adds another level of complexity since this library is generic enough to manipulate anything that relies on forces (CITE https://simtk.org/projects/simbody/). 
The core is therefore much harder to modify, and more importantly would have an impact on all the software that depends on `Simbody` preventing from quick modifications of its stable branch. 
`biorbd` is more straightforward, especially by being less compact---by using explicit code instead of templating---and by being less generic---that is being limited to musculoskeletal modelling.
Although it has its drawbacks, this type of coding is easier to modify and to adapt by the community. 

The second reason is that it is important to implement similar but slightly different tools so they can be cross-validated by the community. 
For instance, two papers (CITE, CITE) recently compared the outputs of `Anybody` and `OpenSim` and came to different results.
Although, the authors provided plausible explanations for this, they had to assume that the implementation of the algorithms are flawless in both software. 
However, due to the closed-source nature of `Anybody`, a direct comparison between the actual codes that produced these differences is impossible.
Having multiple open source software that produces similar ends by different means is a quality assurance for the end users: "Do not put all your eggs in one basket.” 
Therefore, in our view, `biorbd` are `OpenSim` complete more than they compete each other. 

# Acknowledgements
A huge thanks to Ariane Dang for her patience and contribution to writing the tests for `biorbd`!

# References
