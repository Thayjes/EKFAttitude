%% LOAD DATA
% Load the gps and imu tables.
load('F:\Visnav Flight Data\20170815_DPG_Flight1\all.mat');
% Get pos = [x y z], and tgps.
[pos, tgps] = load_gps_meters(gps);
xm = pos(:, 1); ym = pos(:, 2); zm = pos(:, 3);
% Get acceleration and angular rates from the imu as well as the times.
[acc_imu, w_imu, timu] = load_acc_gyro_imu(imu);
[acc_stim, w_stim, tstim] = load_acc_gyro_stim(stim);
ir = table2array(ir);
tir = ir(:, 1);
%% MEASUREMENTS
% Calculate the measurements in the form
% Y = [roll pitch heading t], so at each time we can obtain a measurement
% which can be used to update our current state estimate.
% Y = RPY_measurements(acc_stim, pos, tgps, tstim);
% Here we create velocity and acceleration measurements using the GPS
% position measurements.
ax_stim = acc_stim(:, 1);
ay_stim = acc_stim(:, 2);
az_stim = acc_stim(:, 3);
acc_stim = [ax_stim ay_stim az_stim];
[agps, vgps] = vel_and_acc(pos, tgps);
agps_x = agps(:, 1);
agps_y = agps(:, 2);
agps_z = agps(:, 3);
disp('Number of velocity measurements = '), disp(length(vgps));
disp('Number of acceleration measurements = '), disp(length(agps));
vgps_reduced = vgps(2:end-1, :);
tgps_reduced = tgps(3:end-2);
%% Running the filter
tstim_index = 128845;
tgps_index = 5100;
with_measurement = 1;
% Variable to indicate whether to implement forward and update equations in
% square root form or not
SQRT = 0;
% Variable to indicate whether there will be process noise in q or not
q_noise = 0;
% Create an array to store trajectory of the state. Can be used later.
X = [];
P = {};
% Initialize the quaternion state using the first measurement obtained at
% tgps(1).
% Obtain the initial estimate from the first measurement of roll pitch and
% yaw.
%q_init = [0.6191, 0.2736, 0.0635, 0.7334];
% Define q_init for 20170815_DPG_Flight1 at tgps_index = 5100
q_init = [0.4167, -0.0188, 0.0857, -0.9048];
x_pred = [q_init 0 0 0]'; 
P_pred = diag([0.1 0.1 0.1 0.1 1e-2 1e-2 1e-2]); % Taken from references of the paper
while(tstim_index < 228921)
    tstim_curr = tstim(tstim_index); 
    tgps_curr = tgps_reduced(tgps_index);
    % Check if the stim time has passed the gps time, if yes, then
    % incorporate the "measurement" which has arrived.    
    if(with_measurement == 1)
    if(tstim_curr > tgps_curr)
        %calculate a measurement based on average astim at tstim_index and
        %tstim_index - 1
        % y = [roll pitch yaw]'
        [y] = measure(tgps_index, tstim_index, vgps_reduced, agps, acc_stim);
        % Only if we can obtain a measurement then we run an update
        if(sum(y ~= 0))                        
            [ y_pred ] = measurement_model(x_pred);
            % Update our estimate using the measurement
            [x_updated, P_updated] = EKF_Update(x_pred, P_pred, y_pred, y, SQRT);
            x_updated = normalize_quaternion(x_updated);
            if(tgps_index < length(tgps))
                tgps_index = tgps_index + 1;
            end
            disp('The updated estimate is : '), disp(x_updated);
        else
            % Our best estimate is simply our prediction
            x_updated = x_pred;
            x_updated = normalize_quaternion(x_updated);
            P_updated = P_pred;
            % And we increment our tgps index as this measurement was not
            % good
            if(tgps_index < length(tgps))
                tgps_index = tgps_index + 1;
            end
        end
    else
        % Our best estimate is simply our prediction
        x_updated = x_pred;
        x_updated = normalize_quaternion(x_updated);
        P_updated = P_pred;
    end
    else
        x_updated = x_pred;
        P_updated = P_pred;
    end
    %x_updated = x_pred;
    %P_updated = P_pred;
    X = [X x_updated];
    P = [P P_updated];
    w_curr = w_stim(tstim_index, :)';
    [x_pred, P_pred] = forward_model(x_updated, P_updated, w_curr, SQRT, q_noise);
    if(any(diag(P_pred) < 0) || any(eig(P_pred) < 0))
        disp('Diagonal element of covariance matrix has become negative!');
    end
    x_pred = normalize_quaternion(x_pred);
    tstim_index = tstim_index + 1;
    disp('The predicted estimate is: '), disp(x_pred); 
    tstim_index
end