clc;
clear;
close all;

% This examples shows how to
%     1. Load a model
%     2. Generate data (should be acquired via real data)
%     3. Create a Kalman filter
%     4. Apply the Kalman filter (inverse kinematics)
%     5. Plot the kinematics (Q), velocity (Qdot) and acceleration (Qddot)


% Load a predefined model
model = biorbd('new', '../pyomecaman.bioMod');
nQ = biorbd('nQ', model);
nQdot = biorbd('nQdot', model);
nTau = biorbd('nTau', model);

% Generate random data (3 frames)
nFrames = 3;
targetQ = repmat(rand(nQ, 1), [1, 1, nFrames]);
targetMarkers = biorbd('markers', model, targetQ);

% Perform the kalman filter for each frame at once
[Q, Qdot, Qddot] = biorbd('ik_ekf', model, targetMarkers);
disp(Q - squeeze(targetQ))


% If one is interested in updating the filter by themselves, they can do as
% follow
kalman = biorbd('ik_ekf_new', model);
Q2 = nan(nQ, nFrames);
for i = 1:nFrames
    Q2(:, i) = biorbd('ik_ekf_step', model, kalman, targetMarkers(:, :, i));
end
disp(Q2 - squeeze(targetQ))

% Properly close the model
biorbd('ik_ekf_delete', kalman);
biorbd('delete', model)
