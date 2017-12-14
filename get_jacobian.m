function [ A ] = get_jacobian(x_prev, w)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
q = x_prev(1:4);
b = x_prev(5:7);
A = 0.5*[0 (b(1) - w(1)) (b(2) - w(2)) (b(3) - w(3)) q(2) q(3) q(4);
    (w(1) - b(1)) 0 (w(3) - b(3)) (b(2) - w(2)) -q(1) q(4) -q(3);
    (w(2) - b(2)) (b(3) - w(3)) 0 (w(1) - b(1)) -q(4) -q(1) q(2);
    (w(3) - b(3)) (w(2) - b(2)) (b(1) - w(1)) 0 q(3) -q(2) -q(1);
    0   0   0   0   0   0   0;
    0   0   0   0   0   0   0;
    0   0   0   0   0   0   0];
end

