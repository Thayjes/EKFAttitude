function [y_curr] = measure(t_curr)
%measure This function is used to generate euler angle measurements
%based on the GPS, Accelerometer and Gyroscope data. These measurements
%will be used in the EKF update step.
% Inputs: 1. gps is a mx3 matrix with columns x, y and z all in meters.
%         2. acc is a mx3 matrix with columns ax, ay and az all
%         in m/s^2
%         3. gyro is a mx3 matrix with columns wx, wy and wz all in
%         radians/sec. wx: pitch rate, wy: roll rate, wz: yaw or heading
%         rate.
[gps_acc, gps_vel] = vel_and_acc(gps, t);
agps_x = gps_acc(:, 1); agps_y = gps_acc(:, 2); agps_z = gps_acc(:, 3);
ax = acc(:, 1); ay = acc(:, 2); az = acc(:, 3);
v_x = = gps_vel(:, 1); v_y = gps_vel(:, 2); v_z = gps_vel(:, 3);
avg_vy


end

