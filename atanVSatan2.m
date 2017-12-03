% A small script to test the difference in using atan and atan2 while
% calculating the jacobian for the measurement model.
syms qs qx qy qz bx by bz real
% Here we use atan2
h_atan2 = [atan2( (2*(qy*qz + qs*qx)), (1 - 2*(qx^2 + qy^2)) );
        asin( -2*(qx*qz - qs*qy) );
        atan2( (2*(qx*qy + qs*qz)), (1- 2*(qy^2 + qz^2)))];
H_atan2 = simplify(expand(jacobian(h_atan2, [qs qx qy qz bx by bz])));
disp('H_atan2 = '), disp(H_atan2);

% Here we use atan 
h_atan = [atan( (2*(qy*qz + qs*qx)) / (1 - 2*(qx^2 + qy^2)) );
        asin( -2*(qx*qz - qs*qy) );
        atan( (2*(qx*qy + qs*qz))/(1- 2*(qy^2 + qz^2)))];
H_atan = simplify(expand(jacobian(h_atan, [qs qx qy qz bx by bz])));
disp('H_atan = '), disp(H_atan);