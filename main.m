%% LOAD DATA
% Load the gps and imu tables.
load('F:\Visnav Flight Data\20170719_Upper_Flight1\all.mat');
% Get pos = [x y z], and tgps.
[pos, tgps] = load_gps_meters(gps);
% Get acceleration and angular rates from the imu as well as the times.
[acc_imu, w_imu, timu] = load_acc_gyro_imu(imu);
[acc_stim, w_stim, tstim] = load_acc_gyro_stim(stim);
%% MEASUREMENTS
% Calculate the measurements in the form
% Y = [roll pitch heading t], so at each time we can obtain a measurement
% which can be used to update our current state estimate.
% Y = RPY_measurements(acc_stim, pos, tgps, tstim);
% Here we create velocity and acceleration measurements using the GPS
% position measurements.
[agps, vgps] = vel_and_acc(pos, tgps);
%% Running the filter
tstim_index = 1;
tgps_index = 1;
% Create an array to store trajectory of the state. Can be used later.
X = [];
%x_updated = ??
x_pred = [0 0 0 0 0 0 0]';
P_pred = 0.1*eye(7);
%P_updated = ??

while(tstim_index < length(tstim) - 1)
    tstim_curr = tstim(tstim_index); 
    tgps_curr = tgps(tgps_index);
    % Check if the stim time has passed the gps time, if yes, then
    % incorporate the "measurement" which has arrived.
    if(tstim_curr > tgps_curr)
        %calculate a measurement based on average astim at tstim_index and
        %tstim_index - 1
        % y = [roll pitch yaw]'
        [y] = measure(tgps_index, tstim_index, vgps, agps, acc_stim);
        [ y_pred ] = measurement_model(x_pred);
        % Update our estimate using the measurement
        [x_updated, P_updated] = EKF_Update(x_pred, P_pred, y_pred, y);
        tgps_index = tgps_index + 1;
    else
        % Our best estimate is simply our prediction
        x_updated = x_pred;
        P_updated = P_pred;
    end
    X = [X x_updated];
    w_curr = w_stim(tstim_index, :)';
    [x_pred, P_pred] = forward_model(x_updated, P_updated, w_curr);
    tstim_index = tstim_index + 1;
    disp('The predicted estimate is: '), disp(x_pred);        
end