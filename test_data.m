function [] = test_data(path)
% A script to load gps data and plot the position, velocity, heading and
% acceleration. And then make assessments of the data.
%% LOAD DATA
% Load the gps and imu tables.
load(strcat(path, '\all.mat'));
% Get pos = [x y z], and tgps.
[pos, tgps] = load_gps_meters(gps);
% Get acceleration and angular rates from the imu as well as the times.
[acc_imu, w_imu, timu] = load_acc_gyro_imu(imu);
[acc_stim, w_stim, tstim] = load_acc_gyro_stim(stim);
xm = pos(:, 1); ym = pos(:, 2); zm = pos(:, 3);
index = 1;
index2 = length(tgps);
figure(1)
plot(xm(index:7000), ym(index:7000), 'b-'); title('Position co-ordinates');
[agps, vgps] = vel_and_acc(pos, tgps);
disp('Number of velocity measurements = '), disp(length(vgps));
disp('Number of acceleration measurements = '), disp(length(agps));
vgps_reduced = vgps(2:end-1, :);
tgps_reduced = tgps(3:end-2);
vx = vgps_reduced(:, 1); vy = vgps_reduced(:, 2); vz = vgps_reduced(:, 3);
figure(2)
plot(vx(index:end), 'r-'); title('X Velocity');
figure(3)
plot(vy(index:end), 'r-'); title('Y Velocity');
figure(4)
plot(vz(index:end), 'r-'); title('Z Velocity');
figure(5)
heading = atan2(vy(index:end), vx(index:end));
plot(heading, 'g-'); title('Heading');
ax = agps(:, 1); ay = agps(:, 2); az = agps(:, 3);
figure(6);
plot(ax(index:index2), 'k-'); title('X Acceleration');
figure(7);
plot(ay(index:index2), 'k-'); title('Y Acceleration');
figure(8);
plot(az(index:index2), 'k-'); title('Z Acceleration');
end

