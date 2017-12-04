function [ q ] = toQuaternion(pitch, roll, yaw)
%UNTITLED3 Converts euler angle to quaternion (taken from Wikipedia)
%   Detailed explanation goes here
cy = cos(yaw * 0.5);
sy = sin(yaw * 0.5);
cr = cos(roll * 0.5);
sr = sin(roll * 0.5);
cp = cos(pitch * 0.5);
sp = sin(pitch * 0.5);

qs = cy * cr * cp + sy * sr * sp;
qx = cy * sr * cp - sy * cr * sp;
qy = cy * cr * sp + sy * sr * cp;
qz = sy * cr * cp - cy * sr * sp;
q = [qs;qx;qy;qz];

end

