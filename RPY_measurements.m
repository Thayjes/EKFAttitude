function [ output_args ] = RPY_measurements(acc, gps, t)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
% acc is undefined, t is undefined
[agps, vgps] = vel_and_acc(gps, t);
vx = vgps(:, 1);
vy = vgps(:, 2);
avg_vx = mean([vx(1:end-1)';vx(2:end)'])';
avg_vy = mean([vy(1:end-1)';vy(2:end)'])';
heading = atan(avg_vy./avg_vx);

% Acceleration from gps
agps_x = agps(:, 1);
agps_y = agps(:, 2);
agps_z = agps(:, 3);
% Acceleration from accelerometer
ax = acc(:, 1);
ay = acc(:, 2);
az = acc(:, 3);
% Is the heading already in radians?
rx = -( cos(heading).*agps_x + sin(heading).*agps_y );
ry = -( -sin(heading).*agps_x + cos(heading).*agps_y );
rz = 9.8 - agps_z;


sigma_theta = (rx.*ax + rz.*( (rx.^2 + rz.^2 - ax.^2).^(0.5) ) ) ./ (rx.^2 + rz.^2);
pitch = atan( (sigma_theta.*rx - ax) ./ (sigma_theta.*rz) );
rtheta = rx.*sin(pitch) + rz.*cos(pitch);


sigma_phi = (ry.*ay + rtheta.*((ry.^2 + rtheta.^2 - ay.^2).^(0.5))) ./ (ry.^2 + rtheta.^2);
roll = atan( (-sigma_phi.*ry + ay)/(sigma_theta.*rtheta) );

% We now have the roll, pitch and heading for times t(3:end)
% The reason why it is t(3) is, from t(1:2) we get one velocity and t(2:3)
% we get one more velocity. We then average v1 and v2 to get a vx and vy at
% t = t(3). We then use these to calculate the heading at t = t(3)
end

