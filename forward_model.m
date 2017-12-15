function [x_curr, P_curr] = forward_model(x_prev, P_prev, w, SQRT, q_noise)
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
%4. SQRT: An index (either 0 or 1) indicating whether the filter will use
%square root of P to propagate covariance matrix.
%5. q_noise: An index (0 or 1) indicating whether there will also be
%process noise in the quaternion states (along with the bias states).

% The forward model for the quaternion estimate using Euler's method:
% q(k+1) = q(k) + omega(w(k)-b(k))*q(k)*dt + n_q(k)
% The  forward model for the bias estimate will be a random walk, hence:
% b(k+1) = b(k) + n_b(k)
% UPDATED_EQUATIONS
% x(k+1) = Ad*x(k) + G*B*n(k)
% Where Ad = eye(7) + A*dt , A is the jacobian of f(x, w) at x(k), w(k).
% G = (eye(7)*dt + A*(dt^2/2))
% B = eye(7) or [0; I] depending on q_noise


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
Q = 1e-3*diag([1 1 1 1 1e-2 1e-2 1e-2]);
if(q_noise == 0)
    B = [zeros(4, 7); zeros(3, 4), eye(3)];
else
    B = eye(7);
end
dt = 0.004;
wx = w(1); wy = w(2); wz = w(3); 
q_prev = x_prev(1:4);
b_prev = x_prev(5:7);
bx = b_prev(1); by = b_prev(2); bz = b_prev(3);
n_prev = mvnrnd(zeros(7,1), Q)'; %Process Noise Vector
% Form the omega matrix
omega = 0.5*[0            -(wx - bx)          -(wy - by)          -(wz - bz);
                  (wx - bx)    0                   (wz - bz)           -(wy - by);
                  (wy - by)    -(wz - bz)          0                   (wx - bx);
                  (wz - bz)    (wy - by)           -(wx - bx)          0];
A = get_jacobian(x_prev, w); 
%Define the  state transition matrix using taylor series approximation
F = eye(7) + A*dt; % This is Ad
G = (eye(7)*dt + A*(dt^2)/2);
Gw = G*B;
% Define the discrete noise covariance matrix
Qk = Gw*Q*Gw';
% Implement the forward model equations using euler integration
%q_curr = q_prev + omega*q_prev*dt ;
%b_curr = b_prev;
% Soon-Jo derived equations
x_curr = (eye(7) + [omega, zeros(4,3); zeros(3, 7)]*dt)*x_prev + G*B*n_prev;
%(q_curr - x_curr(1:4));
% Implement the propagation of covariance matrix
%P_curr = F*P_prev*F' + Q*dt % Should this be P_prev + dt*A*P_prev + dt*P_prev*A' + Q*dt??
if(SQRT == 1)
    eig(P_prev)                                                                                                                                                                                                                                                                          
    W = chol(P_prev, 'lower');   
    Sw = chol(Q, 'lower');
    %P_curr = W*W' + dt*A*(W*W') + dt*(W*W')*A' + (Sw*Sw')*dt
    P_curr = F*(W*W')*F' + Qk
else
    %P_curr = P_prev + dt*A*P_prev + dt*P_prev*A' + Q*dt
    P_curr = F*P_prev*F' + Qk
end
% P_curr = 0.5*(P_curr + P_curr'); % To maintain positive definiteness
% x_curr = [q_curr; b_curr];


end

