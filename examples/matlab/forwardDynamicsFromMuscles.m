clc;
clear;
close all;

% This examples shows how to
%     1. Load a model
%     2. Position the model at a chosen position (Q), velocity (Qdot)
%     3. Compute the generalized acceleration (Qddot) assuming a set
%        muscle activations (joint torque from muscle)
%     4. Print them to the console


% Load a predefined model
model = biorbd('new', '../arm26.bioMod');
nQ = biorbd('nQ', model);
nQdot = biorbd('nQdot', model);
nMus = biorbd('nMuscles', model);

% Choose a position/velocity to compute dynamics from
Q = zeros(nQ, 1);
Qdot = zeros(nQdot, 1);

% Set all muscles to half of their maximal activation
emg = ones(nMus, 1) * 0.5;

% Proceed with the computation of joint torque from the muscles
Tau = biorbd('jointTorqueFromActivation', model, emg, Q, Qdot);

% Proceed with the forward dynamics
Qddot = biorbd('forwardDynamics', model, Q, Qdot, Tau);
% Print them to the console
disp(Qddot)

% As an extra, let's print the individual muscle forces
muscleForces = biorbd('muscleForces', model, emg, Q, Qdot);
% Print them to the console
disp(muscleForces)


% Properly close the model
biorbd('delete', model)