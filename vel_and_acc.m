function [agps, vgps] = vel_and_acc(pos, t)
%UNTITLED10 Calculates the acceleration and velocity of the body from the
%GPS masurements.
% The x, y and z co-ordinates are already assumed to be available in meters
% at this stage. We then differentiate the positions to get the velocities.
% We average the velocity over each time interval to get a better estimate.
% We then differentiate the averaged velocity to get the accelerations.
% Which are again averaged for the same reason. 
xm = pos(:, 1);
ym = pos(:, 2);
zm = pos(:, 3);
%xm = xm(5150:end);
%ym = ym(5150:end);
%zm = zm(5150:end);
% Calculate velocity using the position data
vx = diff(xm) ./ diff(t(1:end));
vy = diff(ym) ./ diff(t(1:end));
vz = diff(zm) ./ diff(t(1:end));
avg_vx = mean([vx(1:end-1)';vx(2:end)'])';
avg_vy = mean([vy(1:end-1)';vy(2:end)'])';
avg_vz = mean([vz(1:end-1)';vz(2:end)'])';
% After differencing and averaging velocities we lose the acceleration at two
% instances of time, the last two and first two.
ax = diff(avg_vx) ./ diff(t(2:end-1));
ay = diff(avg_vy) ./ diff(t(2:end-1));
az = diff(avg_vz) ./ diff(t(2:end-1));
avg_ax = mean([ax(1:end-1)';ax(2:end)'])';
avg_ay = mean([ay(1:end-1)';ay(2:end)'])';
avg_az = mean([az(1:end-1)';az(2:end)'])';
%Similarly we lose another two for acceleration, one
%from the end and one from the start. % We can maybe make some assumption of the acceleration at
%those times.
vgps = [avg_vx avg_vy avg_vz];
agps = [avg_ax avg_ay avg_az];
end

