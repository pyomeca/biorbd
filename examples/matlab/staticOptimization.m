clc;
clear;
close all;

% This examples shows how to
%     1. Load a model with muscles
%     2. Position the model at a chosen position (Q) and velocity (Qdot)
%     3. Define a target generalized forces (Tau)
%     4. Compute the muscle activations that reproduce this Tau (Static optimization)
%     5. Print them to the console
%


% Load a predefined model
model = biorbd('new', '../arm26.bioMod');
nQ = biorbd('nQ', model);
nQdot = biorbd('nQdot', model);
nQddot = biorbd('nQddot', model);
nMus = biorbd('nMuscles', model);

% Choose a position/velocity/torque to compute muscle activations from
nFrames = 3;
Q = zeros(nQ, nFrames);
Qdot = zeros(nQdot, nFrames);
Qddot = zeros(nQddot, nFrames);
Tau = biorbd('inverseDynamics', model, Q, Qdot, Qddot);

% Proceed with the static optimization
activations = nan(nMus, nFrames);
prevAct = zeros(nMus, 1);
minAct = zeros(nMus, 1);
maxAct = ones(nMus, 1);
options = optimoptions(@fmincon, 'Display','off');
for i = 1:nFrames
    activations(:, i) = fmincon(@(x)cost(x), prevAct, ...
        [], [], [], [], minAct, maxAct, ...
        @(x)target(x, model, Q(:, i), Qdot(:, i), Tau(:, i)), ...
        options);
    prevAct = activations(:, i);
end

% Print them to the console
disp(activations)

% Verify that these activations produce the desired generalized torques
muscleTau = biorbd('jointTorqueFromActivation', model, activations, Q, Qdot);
disp(muscleTau - Tau)

% Properly close the model
biorbd('delete', model)


function [c, ceq] = target(x, model, q, qdot, tau_target)
    ceq = tau_target - biorbd('jointTorqueFromActivation', model, x, q, qdot);
    c = [];
end

function val = cost(x)
    val = x' * x;
end