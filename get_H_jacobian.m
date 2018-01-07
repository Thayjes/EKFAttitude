function [ H ] = get_H_jacobian( x_prev, model_number)
%get_H_jacobian: This function is used to get the jacobian of the
%non-linear measurement function.
%   Detailed explanation goes here
if(model_number == 1)
q = x_prev(1:4);
qs = q(1); qx = q(2); qy = q(3); qz = q(4);
a = 2*(qs*qx + qy*qz);
c = 1- 2*(qx^2 + qy^2);
dphibydq = ((2*c)/(a^2 + c^2))*[qx; qs + 2*qx*a; qz + 2*qy*a; qy]';
gamma = -2*(qx*qz -qs*qy);
dthetabydq = (2 / sqrt(1- gamma^2))*[qy;-qz;qs;-qx]';
alpha = 2*(qx*qy + qs*qz); beta = 1 - 2*(qy^2 + qz^2);
dpsibydq = ((2*beta)/(alpha^2 + beta^2))*[qz;qy;qx + 2*qy*alpha;qs + 2*qz*alpha]';
% Alternate way to define H
H = [dphibydq zeros(1, 3); dthetabydq zeros(1,3); dpsibydq zeros(1,3)];
%H = [ -(2*qx*(2*qx^2 + 2*qy^2 - 1))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1), (2*(2*qs*qx^2 + 4*qz*qx*qy - 2*qs*qy^2 + qs))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1), (2*(- 2*qz*qx^2 + 4*qs*qx*qy + 2*qz*qy^2 + qz))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1),                   -(2*qy*(2*qx^2 + 2*qy^2 - 1))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1), 0, 0, 0;
%                                                                  (2*qy)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2),                                                                                 -(2*qz)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2),                                                                                    (2*qs)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2),                                                                                   -(2*qx)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2), 0, 0, 0;
% -(2*qz*(2*qy^2 + 2*qz^2 - 1))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1),                 -(2*qy*(2*qy^2 + 2*qz^2 - 1))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1),   (2*(2*qx*qy^2 + 4*qs*qy*qz - 2*qx*qz^2 + qx))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1), (2*(- 2*qs*qy^2 + 4*qx*qy*qz + 2*qs*qz^2 + qs))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1), 0, 0, 0];
if (~isreal(H))
    disp('Measurement Matrix Jacobian is complex!')
end
else
    H = [eye(4,4) zeros(4, 3)];
end
end

