function [ x_normalized ] = normalize_quaternion(x_before)
%UNTITLED Normalize the quaternion vector in the state
%   Detailed explanation goes here
q = x_before(1:4);
q = q / norm(q, 2);
x_normalized = [q;x_before(5:7)];

end

