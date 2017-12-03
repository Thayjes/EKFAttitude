%This script will be used to find the times in the IMU and STIM data which
%match closest to the times of the GPS data. 
% These times will then be used to generate down-samples accelerometer and
% gyroscope values from the IMU and STIM data.
% The downsampled data will be used 

load('F:\Visnav Flight Data\20170719_Upper_Flight1\all.mat');
[pos, tgps] = load_gps_meters (gps);
[acc, w, timu] = load_acc_gyro_imu(imu);
[acc_stim, w_stim, tstim] = load_acc_gyro_stim(stim);
ax = acc(:, 1);
ay = acc(:, 2);
az = acc(:, 3);
axs = acc_stim(:, 1);
ays = acc_stim(:, 2);
azs = acc_stim(:, 3);

[~, timu_locs] = ismembertol(tgps, timu, 1e-8);
[~, tstim_locs] = ismembertol(tgps, tstim, 1e-8);
 
disp('Saving imu locs');
save('timu_locs', 'timu_locs');
disp('Saving stim locs');
save('tstim_locs', 'tstim_locs');
% 
% % Once the locations are obtained we store the accelerometer values at
% % these measurements, we can also get the gyroscope measurements at these
% % times.
ax_imu = ax(timu_locs);
ay_imu = ay(timu_locs);
az_imu = az(timu_locs);

ax_stim = axs(tstim_locs);
ay_stim = ays(tstim_locs);
az_stim = azs(tstim_locs);