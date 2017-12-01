function [agps, vgps] = vel_and_acc(gps, t)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
xm = gps(:, 1);
ym = gps(:, 2);
zm = gps(:, 3);
vx = diff(xm) ./ diff(t);
vy = diff(ym) ./ diff(t);
vz = diff(zm) ./ diff(t);
ax = diff(vx) ./ diff(t(2:end));
ay = diff(vy) ./ diff(t(2:end));
az = diff(vz) ./ diff(t(2:end));
vgps = [vx vy vz];
agps = [ax ay az];
end

