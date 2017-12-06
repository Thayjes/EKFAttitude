function [ discard ] = check_measurement(x_curr, P_curr, y_curr)
%check_measurement This function will be used to validate measurements and
%discard them if they are outisde the dynamic range of the aircraft.
%   Detailed explanation goes here
discard = 0;
q = x_curr(1:4);
q_var = [P_curr(1, 1); P_curr(2, 2); P_curr(3, 3); P_curr(4, 4)];
q_sigma = sqrt(q_var);
q_measured = toQuaternion(y_curr(1), y_curr(2), y_curr(3));
if (abs(q_measured- q_curr) > 3*q_sigma)
    discard = 1;
end

end

