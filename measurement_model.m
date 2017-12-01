function [ y_hat ] = measurement_model(x_prev)
%measurement_model Describes the non-linear measurement model.
%   Detailed explanation goes here
q = x_prev(1:4);
y_hat = [atan( (2*(q(3)*q(4) + q(1)*q(2))) / (1 - 2*(q(2)^2 + q(3)^2)) );
         asin( -2*(q(2)*q(4) - q(1)*q(3)) );
         atan( (2*(q(2)*q(3) + q(1)*q(4))) / (1 - 2*(q(3)^2 + q(4)^2)) )];
% roll = y_hat(1)
% pitch = y_hat(2)
% yaw/heading = y_hat(3)

end

