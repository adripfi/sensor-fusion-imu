function e = cost_func_43(x)
    
    global quatC_glob
    global om_rel
    global j2_world
    
    % spherical coord. to cartesian
    j1 = [sin(x(1))*cos(x(2)) sin(x(1))*sin(x(2)) cos(x(1))];
    % j1 in world frame
    j1_world = quaternionRotate(quatC_glob, j1);
    
    % error vector
    e = dot(om_rel, cross(j1_world, j2_world, 2), 2);   
end
