function [F,u,h,H] = ekfDerivation()
    % Calculation of the Matrices with symbolic expression
    syms w x y z bx by bz ux uy uz T
    % state vector
    x_k = [w x y z bx by bz];

    % gyroscope measurement vector
    u_norm = norm([ux uy uz]- [bx by bz]);
    % gyroscope measurement quaternion
    u_k = [cos(1/2*u_norm*T)
         sin(1/2*u_norm*T)*(ux - bx)/u_norm
         sin(1/2*u_norm*T)*(uy - by)/u_norm
         sin(1/2*u_norm*T)*(uz - bz)/u_norm];   
    % predicition function
    f_x = [w*u_k(1) - x*u_k(2) - y*u_k(3) - z*u_k(4)
           w*u_k(2) + x*u_k(1) + y*u_k(4) - z*u_k(3)
           w*u_k(3) + y*u_k(1) - x*u_k(4) + z*u_k(2)
           w*u_k(4) + z*u_k(1) + x*u_k(3) - y*u_k(2)
           bx; by; bz];                
    % prediction jacobian
    F_k = jacobian(f_x, x_k);
    
    % measurement function using ref acc and mag vector
    h_x = [quaternionRotate(quaternionInvert(x_k(1:4)), [0 0 9.81]) quaternionRotate(quaternionInvert(x_k(1:4)), [0 1 0])];
    % measurement jacobian
    H_k = jacobian(h_x, x_k);
    
    
       
    % Create Matlab functions for symbolic jacobians and functions
    u = matlabFunction(u_k);
    h = matlabFunction(h_x);
    H = matlabFunction(H_k);
    F = matlabFunction(F_k);    
end