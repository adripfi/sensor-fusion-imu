function [q] = quaternionFromRotmat(R)
%QUATERNIONFROMROTMAT Creates a quaternion from a rotation matrix.

    w_sq = (1 + R(1,1) + R(2,2) + R(3,3)) / 4;
    x_sq = (1 + R(1,1) - R(2,2) - R(3,3)) / 4;
    y_sq = (1 - R(1,1) + R(2,2) - R(3,3)) / 4;
    z_sq = (1 - R(1,1) - R(2,2) + R(3,3)) / 4;

    w = sqrt(w_sq);
    x = copysign(sqrt(x_sq), R(3,2) - R(2,3));
    y = copysign(sqrt(y_sq), R(1,3) - R(3,1));
    z = copysign(sqrt(z_sq), R(2,1) - R(1,2));
    
    q = [w x y z];
end
function [r] = copysign(x, y)
    s = sign(y);
    if s == 0
        s = 1;
    end
    r = abs(x)*s;
end
