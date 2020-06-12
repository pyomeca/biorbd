clc;
clear;
close all;

% This examples shows how to
%     1. Load a model
%     2. Position the model at a chosen position (Q), velocity (Qdot)
%     3. Compute the generalized acceleration (Qddot) assuming a set of 
%        generalized forces (forward dynamics)
%     4. Print them to the console


% Load a predefined model
model = biorbd('new', '../pyomecaman.bioMod');
nQ = biorbd('nQ', model);
nQdot = biorbd('nQdot', model);
nTau = biorbd('nTau', model);

% Choose a position/velocity/torque to compute dynamics from
Q = zeros(nQ, 1);
Qdot = zeros(nQdot, 1);
Tau = zeros(nTau, 1);

% Proceed with the forward dynamics
Qddot = biorbd('forwardDynamics', model, Q, Qdot, Tau);

% Print them to the console
disp(Qddot)

% Properly close the model
biorbd('delete', model)