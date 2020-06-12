clc;
clear;
close all;

% This examples shows how to
%     1. Load a model
%     2. Position the model at a chosen position (Q), velocity (Qdot) and acceleration (Qddot)
%     3. Compute the generalized forces (tau) at this state (inverse dynamics)
%     4. Print them to the console


% Load a predefined model
model = biorbd('new', '../pyomecaman.bioMod');
nQ = biorbd('nQ', model);
nQdot = biorbd('nQdot', model);
nQddot = biorbd('nQddot', model);

% Choose a position/velocity/acceleration to compute dynamics from
Q = zeros(nQ, 1);
Qdot = zeros(nQdot, 1);
Qddot = zeros(nQddot, 1);

% Proceed with the inverse dynamics
Tau = biorbd('inverseDynamics', model, Q, Qdot, Qddot);

% Print them to the console
disp(Tau)

% Properly close the model
biorbd('delete', model)