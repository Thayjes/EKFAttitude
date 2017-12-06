function [y_curr] = measure(tgps_index, tstim_index, vgps, agps, astim)
%measure This function is used to generate euler angle measurements
%based on the GPS, Accelerometer and Gyroscope data. These measurements
%will be used in the EKF update step.
% Inputs: 1. tgps_index is the index to get the gps time.
%         2. tstim_index is the index to get the stim time.
%         3. vgps is the velocity derived from the GPS position
%         measurement.
%         4. agps is the acceleration derived from the GPS position
%         measurement. 
%         5. astim is the acceleration from the STIM.
% We average the values from the current and previous time step.
disp('The gps time index is: '), disp(tgps_index);
disp('The stim time index is: '), disp(tstim_index);
astim_x = (astim(tstim_index, 1) + astim(tstim_index - 1, 1))/2;
astim_y = (astim(tstim_index, 2) + astim(tstim_index - 1, 2))/2;
vy = vgps(tgps_index, 1);
vx = vgps(tgps_index, 2);
heading = atan2(vy, vx);
agps_x = agps(tgps_index, 1);
agps_y = agps(tgps_index, 2);
agps_z = agps(tgps_index, 3);

rx = -( cos(heading)*agps_x + sin(heading)*agps_y );
ry = -( -sin(heading)*agps_x + cos(heading)*agps_y );
rz = 9.8 - agps_z;


sigma_theta = (rx*astim_x + rz*( (rx^2 + rz^2 - astim_x.^2)^(0.5) ) ) / (rx^2 + rz^2);
if(isreal(sigma_theta))
    pitch = atan2( (sigma_theta*rx - astim_x), (sigma_theta*rz) );
    rtheta = rx*sin(pitch) + rz*cos(pitch);

    
    sigma_phi = (ry*astim_y + rtheta*((ry^2 + rtheta^2 - astim_y^2)^(0.5))) / (ry^2 + rtheta^2);
    if(isreal(sigma_phi))
        roll = atan( (-sigma_phi*ry + astim_y)/(sigma_theta*rtheta) );
        y_curr = [roll;pitch;heading];
    else
        disp('Sigma_Phi is complex!');
        y_curr = [0;0;0];
    end
    
else
    disp('Sigma_Theta is complex!');
    y_curr = [0;0;0];
end

end

