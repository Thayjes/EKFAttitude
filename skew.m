function [skew_matrix] = skew(w, b)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
if nargin == 1
    skew_matrix = [0 -w(1) -w(2);
                   w(1) 0 w(3);
                   w(2) - w(3) 0;];
else
    skew_matrix = [0 -(w(1) - b(1)) -(w(2) - b(2));
                   (w(1) - b(1)) 0 (w(3) - b(3));
                   (w(2) - b(2)) -(w(3) - b(3)) 0];
end

