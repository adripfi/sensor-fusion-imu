function [quat_curr_prev] = quaternionFromGyr(gyr, rate)
%QUATERNIONFROMGYR Create a quaternion from a gyroscope sample.
    
    gyrNorm = norm(gyr);
    
    axis = gyr / gyrNorm; %TODO
    angle = gyrNorm / rate; % TODO
    quat_curr_prev = [cos(angle/2) axis*sin(angle/2)]; % TODO
end

