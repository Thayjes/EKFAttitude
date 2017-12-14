function [Y, yaw_measured, roll_measured, pitch_measured] = RPY_measurements(acc_stim, pos, tgps, tstim)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
% acc is undefined, t is undefined
% We calculate the velocity and acceleration using gps position
% measurements.

[agps, vgps] = vel_and_acc(pos, tgps);
disp('Number of velocity measurements = '), disp(length(vgps));
disp('Number of acceleration measurements = '), disp(length(agps));
vgps_reduced = vgps(2:end-1, :);
tgps_reduced = tgps(3:end-2);
tstim_index = 1;
tgps_index = 1;
Y = [];
while(tstim_index < length(tstim) - 1)
    tstim_curr = tstim(tstim_index); 
    tgps_curr = tgps_reduced(tgps_index);
    % Check if the stim time has passed the gps time, if yes, then
    % incorporate the "measurement" which has arrived.    
    if(tstim_curr > tgps_curr)
        %calculate a measurement based on average astim at tstim_index and
        %tstim_index - 1
        % y = [roll pitch yaw]'
        [y] = measure(tgps_index, tstim_index, vgps_reduced, agps, acc_stim); 
        Y = [Y; y' tgps_index];
        tgps_index = tgps_index + 1;
    else
        tstim_index = tstim_index + 1;
    end
    if(tgps_index == length(tgps_reduced))
        break
    end
end
yaw_measured = rad2deg(Y(:, 3));
roll_measured = rad2deg(Y(:, 1));
pitch_measured = rad2deg(Y(:, 2));
a = Y(:, 3);
complex = find(a == 0);
disp('The number of complex measurements is: '); disp(length(complex));
% % The velocity is already averaged.
% heading = atan2(vy, vx);
% 
% % Acceleration from gps
% agps_x = agps(:, 1);
% agps_y = agps(:, 2);
% agps_z = agps(:, 3);
% % Acceleration from accelerometer
% % Unfortunately the accelerometer data is obtained at a much higher
% % frequency than the GPS acceleration measurements. We need to downsample
% % the accelerometer measurements based on the time stamp.
% load('tstim_locs');
% ax = acc(:, 1);
% ay = acc(:, 2);
% az = acc(:, 3);
% ax_sampled = ax((tstim_locs));
% ax_sampled = ax_sampled(3:end-2);
% ay_sampled = ay((tstim_locs));
% ay_sampled = ay_sampled(3:end-2);
% az_sampled = az(tstim_locs);
% % Is the heading already in radians?
% rx = -( cos(heading).*agps_x + sin(heading).*agps_y );
% ry = -( -sin(heading).*agps_x + cos(heading).*agps_y );
% rz = 9.8 - agps_z;
% 
% 
% sigma_theta = (rx.*ax_sampled + rz.*( (rx.^2 + rz.^2 - ax_sampled.^2).^(0.5) ) ) ./ (rx.^2 + rz.^2);
% pitch = atan2( (sigma_theta.*rx - ax_sampled), (sigma_theta.*rz) );
% rtheta = rx.*sin(pitch) + rz.*cos(pitch);
% 
% 
% sigma_phi = (ry.*ay_sampled + rtheta.*((ry.^2 + rtheta.^2 - ay_sampled.^2).^(0.5))) ./ (ry.^2 + rtheta.^2);
% roll = atan( (-sigma_phi.*ry + ay_sampled)/(sigma_theta.*rtheta) );
% 
% % We now have the roll, pitch and heading for times t(3:end-2)
% Y = [roll pitch heading tgps(3:end-2)];
end

