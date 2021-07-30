function [quat] = estimateQuaternion(acc, gyr, mag, rate, tauAcc, tauMag, zeta, accRating)
    N = size(acc, 1);
    state = zeros(1, 18);
    state(1) = 1;
    for i=1:N
        [state, errorAngleIncl, errorAngleAzi] = CSG_OriEst_IMU(state, acc(i,:), gyr(i,:), mag(i,:), rate, tauAcc, tauMag, zeta, accRating);
        quat(i,:) = state(1:4);
    end
end

