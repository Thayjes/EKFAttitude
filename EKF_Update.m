function [ x_updated, P_updated ] = EKF_Update(x_curr, P_curr, y_hat, y_curr)
%EKF_Update This function uses a measurement to update the current estimate
%of the state.
%   Detailed explanation goes here
%Parameters (TBD):
%1. R: Measurement Noise Covariance Matrix
H = get_H_jacobian(x_curr);
K = P_curr*H'/(H*P_curr*H' + R);
x_updated = x_curr + K*(y_curr - y_hat);
P_updated = (eye(7) - K*H)*P_curr;
end

