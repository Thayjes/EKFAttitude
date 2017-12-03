function [ H ] = get_H_jacobian( x_prev )
%get_H_jacobian: This function is used to get the jacobian of the
%non-linear measurement function.
%   Detailed explanation goes here
q = x_prev(1:4);
qs = q(1); qx = q(2); qy = q(3); qz = q(4);

H = [ -(2*qx*(2*qx^2 + 2*qy^2 - 1))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1), (2*(2*qs*qx^2 + 4*qz*qx*qy - 2*qs*qy^2 + qs))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1), (2*(- 2*qz*qx^2 + 4*qs*qx*qy + 2*qz*qy^2 + qz))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1),                   -(2*qy*(2*qx^2 + 2*qy^2 - 1))/(4*qs^2*qx^2 + 8*qs*qx*qy*qz + 4*qx^4 + 8*qx^2*qy^2 - 4*qx^2 + 4*qy^4 + 4*qy^2*qz^2 - 4*qy^2 + 1), 0, 0, 0;
                                                                  (2*qy)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2),                                                                                 -(2*qz)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2),                                                                                    (2*qs)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2),                                                                                   -(2*qx)/(- 4*qs^2*qy^2 + 8*qs*qx*qy*qz - 4*qx^2*qz^2 + 1)^(1/2), 0, 0, 0;
 -(2*qz*(2*qy^2 + 2*qz^2 - 1))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1),                 -(2*qy*(2*qy^2 + 2*qz^2 - 1))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1),   (2*(2*qx*qy^2 + 4*qs*qy*qz - 2*qx*qz^2 + qx))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1), (2*(- 2*qs*qy^2 + 4*qx*qy*qz + 2*qs*qz^2 + qs))/(4*qs^2*qz^2 + 8*qs*qx*qy*qz + 4*qx^2*qy^2 + 4*qy^4 + 8*qy^2*qz^2 - 4*qy^2 + 4*qz^4 - 4*qz^2 + 1), 0, 0, 0];

end

