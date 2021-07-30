function [qInvert] = quaternionInvert (q)
    if (size(q, 2) ~= 4)
        error('input has to be Nx4')
    end
    qInvert = [q(:,1) -q(:,2:4)];
end
