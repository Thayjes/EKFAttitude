function [RPY_radian, RPY_deg, b] = to_euler_angles(X)
%to_euler_angles Takes the state trajectory and extracts roll, pitch, yaw
% in radians and degrees. Also extracts the gyroscope bias estimates.
%   Detailed explanation goes here
qs = X(1, :); qx = X(2, :); qy = X(3, :); qz = X(4, :);
bx = X(5, :); by = X(6, :); bz = X(7, :);
roll = atan2( (2*(qy.*qz + qs.*qx)), (1- 2*(qx.^2 + qy.^2)) );
roll_deg = rad2deg(roll);
pitch = asin( -2*(qx.*qz - qs.*qy) );
pitch_deg = rad2deg(pitch);
yaw = atan2( (2*(qx.*qy + qs.*qz)), (1 - 2*(qy.^2 + qz.^2)) );
yaw_deg = rad2deg(yaw);
RPY_radian = [roll' pitch' yaw'];
RPY_deg = [roll_deg' pitch_deg' yaw_deg'];
b = [bx' by' bz'];
end

