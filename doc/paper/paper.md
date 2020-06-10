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
 - name: École de Kinésiologie et de Sciences de l'Activité Physique, Université de Montréal
   index: 1
date: May 1st, 2020
bibliography: paper.bib
---

# Summary
Biomechanics is at the interface of several fields of science, such as robotics, mechanics and human physiology.
Although this transdisciplinarity encourages the emergence of new ideas, the variety of data to analyze simultaneously can be overwhelming.
Commonly biomechanical datasets are composed of skin markers trajectories (SMT), contact forces, electromyographic (EMG) signal, inertial measurement units (IMU) kinematics, etc., which by nature are not straightforward to combine.
It is at their meeting point---the body movement---that `biorbd` steps in: `bio` standing for biomechanics and `rbd` for `rigid body dynamics`.
`biorbd` is a *feature-based development* library that targets the manipulation of biomechanical data in a comprehensive and accessible manner.
For a given musculoskeletal model, it provides functions for inverse flow---i.e. from SMT to EMG---and direct flow---i.e. from EMG to SMT.

Biomechanics often requires computationally expensive or real-time computations.
This is why the core of `biorbd` is written in C++.
Although this language provides fast computations, it lacks the flexibility of higher-level languages.
To meet the needs of the biomechanics community, Python and MATLAB binders are provided with `biorbd`.
As a result, `biorbd` can be elegantly incorporated in usual workflows of researchers without compromising the required speed.

Finally, biomechanical data are often multidimensional and almost always time-dependent which can be challenging to visualize.
`BiorbdViz` [@Michaud2018biorbdViz], a Python visualizer, was purposely-designed.
This visualizer allows to animate the model, record videos, and, for models that include muscles, plot muscular outputs against various features of the movement.

# A `biorbd` overview, the inverse and direct flow
Biomechanical analyses are usually based on one (or a mixture) of the inverse or direct flow [@kainzJointKinematicCalculation2016].
Briefly, the former uses measurements from a movement (e.g., the SMT) and infers its cause, while the latter assumes a control (e.g., EMG) and outputs the resulting kinematics.

## Inverse flow
*Inverse kinematics*: Estimates the generalized coordinates ($q$)—i.e. the body kinematics—from body sensor measurements (e.g., SMT, IMU, etc.).
The main algorithm implemented is the Extended Kalman Filter [@fohannoEstimation3DKinematics2010] which by design facilitates the merging of multiple data sources and takes care of missing data.

*Inverse dynamics*: Estimates the generalized forces ($\tau$) producing a given generalized acceleration ($\ddot{q}$) (the second time derivative of $q$):
$$
\tau = M(q)\ddot{q} + N(q, \dot{q})
$$
where $\dot{q}$ is the generalized velocities, $M(q)$ is the mass matrix and $N(q, \dot{q})$ are the nonlinear effects.

*Static optimization*: Estimates the muscle activations ($\alpha$) producing a given $\tau$ [@andersonStaticDynamicOptimization2001b].
It minimizes the muscle activation *p*-norm ($p$ usually being $2$) that matches a given $\tau$ using nonlinear optimization (Ipopt [@wachterImplementationInteriorpointFilter2006a]).
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
Multiple activation/excitation dynamics are implemented (e.g. @thelenAdjustmentMuscleMechanics2003 and @manalOneparameterNeuralActivation2003).

*Muscular joint torque*: Estimates the $\tau_{mus}$ from muscle forces ($F_{mus}(q, \dot{q}, \alpha)$) [@shermanHowComputeMuscle2010], estimated from $\alpha$ using a muscle model (e.g.,  @hillHeatShorteningDynamic1938, @thelenAdjustmentMuscleMechanics2003):
$$
\tau_{mus} = J_{mus}(q)^T F_{mus}(q, \dot{q}, \alpha)
$$
where $J_{mus}(q)$ is the muscle lengths Jacobian.

*Forward dynamics*: Estimates the $\ddot{q}$ from a given $\tau$:
$$
\ddot{q} = M(q)^{-1}\tau - N(q, \dot{q})
$$
All the forward dynamics implemented in `RBDL` [@felisRBDLEfficientRigidbody2017] are available.

*Forward kinematics*: Estimates the model kinematics outputs (e.g., SMT, IMU) from a given $q$, after integrating twice $\ddot{q}$.

# The dependencies
`biorbd` takes advantage of efficient back ends, especially  the `RBDL` and `CasADi` libraries.
`RBDL`, written by Martin Feliz [@felisRBDLEfficientRigidbody2017], implements the Featherstone's equations of spatial geometry [@featherstoneRobotDynamicsEquations2000], successfully used in the field of robotics [@macchiettoMomentumControlBalance2009; @diehlFastDirectMultiple2006; @kurfessRoboticsAutomationHandbook2018]. 
RBDL provides the computational core for body dynamics.
`biorbd` extends `RBDL` by giving commonly used  biomechanics nomenclature, and adding a muscle module, amongst others. 
`RBDL` is based on the highly efficient C++ linear algebra library `Eigen` [@eigenweb].

Although `Eigen` is flexible and fast enough for most of the common usage, it can’t automatically provide derivatives of functions.
Therefore, `RBDL` was also augmented with the algorithmic differentiation library `CasADi` [@Andersson2018].
`CasADi`allows to compute at low cost the derivatives of almost all the functions in `RBDL`.
This is particularly useful when using `biorbd` in a gradient-based optimization setting.

# The already-existing software in biomechanics
`OpenSim` [@sethOpenSimSimulatingMusculoskeletal2018] and `Anybody` [@damsgaardAnalysisMusculoskeletalSystems2006] are state-of-the-art biomechanics software that provide similar analysis flows with advanced user interface.
`Anybody` being a closed and proprietary software, the reason to create another library for the open-source community is self-explanatory.
Conversely, `OpenSim` is open-source and well established in the biomechanics community.
There are two main reasons that explain the need for `biorbd`.

The second reason is that it is important to implement similar but slightly different tools so they can be cross-validated by the community.
For instance, two papers (@kimSimilaritiesDifferencesMusculoskeletal2018; @trinlerMuscleForceEstimation2019b) recently compared the outputs of `Anybody` and `OpenSim` and came to different results.
Although the authors provided plausible explanations for this, they had to assume that the implementation of the algorithms are flawless in both software.
However, due to the closed-source nature of `Anybody`, a direct comparison between the actual codes that produced these differences is impossible.
Having multiple open source software that produces similar ends by different means is a quality assurance for the end users: "Do not put all your eggs in one basket.”
This is in line with the idea that simulation software should be validated in multiple ways [@hicksMyModelGood2015].
As far as we know, there is no other open source software that provides a complete direct and inverse flow in biomechanics. 
Therefore, in our opinion, `biorbd` and `OpenSim` are more complementary than competitive.

# Acknowledgements
A huge thanks to Ariane Dang for her patience and contribution to writing the tests for `biorbd`!

# References
