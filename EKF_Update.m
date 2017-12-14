function [ x_updated, P_updated ] = EKF_Update(x_curr, P_curr, y_hat, y_curr, SQRT)
%EKF_Update This function uses a measurement to update the current estimate
%of the state.
%   Detailed explanation goes here
%Parameters (TBD):
%1. R: Measurement Noise Covariance Matrix
dt = 0.004;
cholesky = 0;
R = 0.1*eye(3) / dt; %Might have to decrease this
H = get_H_jacobian(x_curr); %%Check the jacobian, imaginary values?
if(~isreal(H))
    disp('Matrix Jacobian of measurement is complex!');
end
if(SQRT == 1)
    W = chol(P_curr, 'lower');
    S = (H*(W*W')*H' + R);
else
    S = (H*P_curr*H' + R);
end


if cholesky == 1
    L = chol(S);
    L = L + 1e-2*eye(length(L));
    u = (L')\eye(length(L));
    Sinv = L\u;
    if(SQRT == 1)
        K = (W*W')*H'*Sinv;
    else
        K = P_curr*H'*Sinv;
    end
else
    if(SQRT == 1)
        K = (W*W')*H'/S;
    else
        K = P_curr*H'/S;
    end
end
n_v = mvnrnd(zeros(3,1), R)';
x_updated = double(x_curr + K*(y_curr - y_hat));
% Use joseph form of covariance update
if(SQRT == 1)
    P_updated = (eye(7) - K*H)*(W*W')*(eye(7) - K*H)' + K*R*K';
else
    P_updated = (eye(7) - K*H)*(P_curr)*(eye(7) - K*H)' + K*R*K';
end
end

