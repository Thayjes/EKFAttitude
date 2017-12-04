function [x_curr, P_curr] = forward_model(x_prev, P_prev, w)
%forward_model: This function is used to march the model forward in time to
%obtain estimates of the attitude and gyroscope bias at required times.
% Parameters (to be defined):
%1. Q: Process Noise Covariance Matrix
%2. dt: The time step
%%Inputs:
%1. x_prev: A 7x1 state vector containing q_prev and b_prev.
%   q_prev: A 4x1 vector containing the current quaternion estimate of attitude.
%   b_prev: A 3x1 vector containing the current estimate of the gyroscope bias.
%2. P_prev: A 7x7 covariance matrix of the state x_prev.
%3. w: A 3x1 vector of angular rates measured by the gyros containing the
%the roll, pitch and yaw rates in that order.

% The forward model for the quaternion estimate using Euler's method:
% q(k+1) = q(k) + omega(w(k)-b(k))*q(k)*dt + n_q(k)
% The  forward model for the bias estimate will be a random walk, hence:
% b(k+1) = b(k) + n_b(k)

%omega(w(k)-b(k)) = 0.5*[0            -(wx - bx)          -(wy - by)          -(wz - bz);
%                  (wx - bx)    0                   (wz - bz)           -(wy - by);
%                  (wy - by)    -(wz - bz)          0                   (wx - bx);
%                  (wz - bz)    (wy - by)           -(wx - bx)          0];
% wx = roll rate, wy = pitch rate, wz = yaw rate
% w(k) = [wx wy wz]'
% q(k) = [qs qx qy qz]'
% b(k) = [bx by bz]'
% n_q(k) = process noise vector for q(k)
% n_b(k) = process noise vector for b(k)
Q = 0.01*eye(7);
dt = 0.04;
q_prev = x_prev(1:4);
b_prev = x_prev(5:7);
n_prev = mvnrnd(zeros(7,1), Q)'; %Process Noise Vector
n_q = n_prev(1:4);
n_b = n_prev(5:7);
skew_matrix = skew(w, b_prev);
%The skew matrix is the the first 3 rows and columns of omega
w_corr = w - b_prev; %The corrected angular rates
%This is just used to form the last column and row of the omega matrix
w_rev = flipud(w_corr); 
w_rev(3) = -w_rev(3);
% Form the omega matrix
omega = 0.5*[skew_matrix w_rev; -w_rev' 0];

% Implement the forward model equations
q_curr = q_prev + omega*q_prev*dt; %+ n_q; % Is there n_q here?
b_curr = b_prev; %+ n_b;
% Implement the propagation of covariance matrix
A = get_jacobian(x_prev, w);
%Define the  state transition matrix using taylor series approximation
F = eye(7) + A*dt;
P_curr = F*P_prev*F' + Q*dt

x_curr = [q_curr; b_curr];


end

