%Save Results
model_number = 1;
Q = 1*diag([1e-4 1e-4 1e-4 1e-4 1e-2 1e-2 1e-2]);
P_init = diag([0.1 0.1 0.1 0.1 1e-2 1e-2 1e-2]);
[Y, yaw_measured, roll_measured, pitch_measured] = RPY_measurements(acc_stim, pos, tgps, tstim, model_number);
[yaw_upd, roll_upd, pitch_upd] = to_euler_angles(X);
figure(1); plot(yaw_upd, 'r-');figure(2); plot(yaw_measured(5100:9100), 'b-');
figure(3); plot(roll_upd, 'r-'); figure(5); plot(roll_measured(5100:9100), 'b-');
figure(4); plot(pitch_upd, 'r-'); figure(7); plot(pitch_measured(5100:9100), 'b-');
dt = 0.004;
R = 0.001*eye(4) / dt;
save('/Users/thayjessrivas/Documents/Attitude_Estimation_Results/Quaternion_Model/EKF_128845_228921_Model2_q&bNonoise_Results', 'X', 'P_init', 'Q', 'R', 'dt');