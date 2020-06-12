clc;
clear;
close all;

% This examples shows how to
%     1. Load a model
%     2. Position the model at a chosen position (Q)
%     3. Compute the position of the skin markers at that position (Forward kinematics)
%     4. Print them to the console


% Load a predefined model
model = biorbd('new', '../pyomecaman.bioMod');
nQ = biorbd('nQ', model);

% Choose a position to get the markers from
Q = zeros(nQ, 1);

% Proceed with the forward kinematics
markers = biorbd('markers', model, Q);

% Print them to the console
disp(markers)

% Properly close the model
biorbd('delete', model)
