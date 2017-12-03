function [acc, w, tstim] = load_acc_gyro_stim(stim)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% Get the accelerometer values
ax = table2array(stim(:, 6));
ay = table2array(stim(:, 7));
az = table2array(stim(:, 8));
acc = [ax ay az];
% Should the angular rates be taken from imu or STIM? Adjust accordingly
wx = table2array(stim(:, 11));
wy = table2array(stim(:, 10));
wz = table2array(stim(:, 9));
tstim = table2array(stim(:, 1));
w = [wx wy wz];

end

