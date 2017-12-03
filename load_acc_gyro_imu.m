function [acc, w, timu] = load_acc_gyro_imu(imu)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%NOTE: If we are using STIM data we have to change imu to stim and adjust
%the column numbers in the table.
% Get the accelerometer values
ax = table2array(imu(:, 4));
ay = table2array(imu(:, 5));
az = table2array(imu(:, 6));
acc = [ax ay az];
% Should the angular rates be taken from imu or STIM? Adjust accordingly
wx = table2array(imu(:, 9));
wy = table2array(imu(:, 8));
wz = table2array(imu(:, 7));
timu = table2array(imu(:, 1));
w = [wx wy wz timu];

end

