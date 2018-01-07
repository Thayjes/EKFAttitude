function [ y_hat ] = measurement_model(x_prev, model_number)
%measurement_model Describes the non-linear measurement model.
%   Detailed explanation goes here
% If model_number is 1, we use euler angles as measurements
% If model_number is 2, we use quaternion as measurements
q = x_prev(1:4);
if(model_number == 1)
    y_hat = [atan2( (2*(q(3)*q(4) + q(1)*q(2))), (1 - 2*(q(2)^2 + q(3)^2)) );
            asin( -2*(q(2)*q(4) - q(1)*q(3)) );
            atan2( (2*(q(2)*q(3) + q(1)*q(4))), (1 - 2*(q(3)^2 + q(4)^2)) )];
else
    y_hat = q;
end
% roll = y_hat(1)
% pitch = y_hat(2)
% yaw/heading = y_hat(3)

end

