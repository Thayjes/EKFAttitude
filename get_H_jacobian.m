function [ H ] = get_H_jacobian( x_prev )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
q = x_prev(1:4);
qs = q(1); qx = q(2); qy = q(3); qz = q(4);
%First row
H(1,1) = -(2*qx)/(((2*qs*qx + 2*qy*qz)^2/(2*qx^2 + 2*qy^2 - 1)^2 + 1)*(2*qx^2 + 2*qy^2 - 1));
H(1,2) = -((2*qs)/(2*qx^2 + 2*qy^2 - 1) - (4*qx*(2*qs*qx + 2*qy*qz))/(2*qx^2 + 2*qy^2 - 1)^2)/((2*qs*qx + 2*qy*qz)^2/(2*qx^2 + 2*qy^2 - 1)^2 + 1);
H(1,3) = -((2*qz)/(2*qx^2 + 2*qy^2 - 1) - (4*qy*(2*qs*qx + 2*qy*qz))/(2*qx^2 + 2*qy^2 - 1)^2)/((2*qs*qx + 2*qy*qz)^2/(2*qx^2 + 2*qy^2 - 1)^2 + 1);
H(1,4) = -(2*qy)/(((2*qs*qx + 2*qy*qz)^2/(2*qx^2 + 2*qy^2 - 1)^2 + 1)*(2*qx^2 + 2*qy^2 - 1));
H(1, 5:7) = [0 0 0];

%Second row
H(2, 1) = (2*qy)/(1 - (2*qs*qy - 2*qx*qz)^2)^(1/2);
H(2, 2) = -(2*qz)/(1 - (2*qs*qy - 2*qx*qz)^2)^(1/2);
H(2, 3) = (2*qs)/(1 - (2*qs*qy - 2*qx*qz)^2)^(1/2);
H(2, 4) = -(2*qx)/(1 - (2*qs*qy - 2*qx*qz)^2)^(1/2);
H(2, 5:7) = [0 0 0];

%Third row
H(3, 1) = -(2*qz)/(((2*qs*qz + 2*qx*qy)^2/(2*qy^2 + 2*qz^2 - 1)^2 + 1)*(2*qy^2 + 2*qz^2 - 1));
H(3, 2) = -(2*qy)/(((2*qs*qz + 2*qx*qy)^2/(2*qy^2 + 2*qz^2 - 1)^2 + 1)*(2*qy^2 + 2*qz^2 - 1));
H(3, 3) = -((2*qx)/(2*qy^2 + 2*qz^2 - 1) - (4*qy*(2*qs*qz + 2*qx*qy))/(2*qy^2 + 2*qz^2 - 1)^2)/((2*qs*qz + 2*qx*qy)^2/(2*qy^2 + 2*qz^2 - 1)^2 + 1);
H(3, 4) = -((2*qs)/(2*qy^2 + 2*qz^2 - 1) - (4*qz*(2*qs*qz + 2*qx*qy))/(2*qy^2 + 2*qz^2 - 1)^2)/((2*qs*qz + 2*qx*qy)^2/(2*qy^2 + 2*qz^2 - 1)^2 + 1);
H(3, 5:7) = [0 0 0];

end

