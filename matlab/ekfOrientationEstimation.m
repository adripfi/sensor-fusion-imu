% 9D EKF for orientation estimation
% The filter additionaly estimates the bias of the gyroscope measurements

function [quat,bias] = ekfOrientationEstimation(acc, gyr, mag, rate)
    N = size(mag, 1);
    quat = zeros(N, 4);
    
    % init estimate
    quat(1,:) = estimateQuaternion(acc(1,:), gyr(1,:), mag(1,:), rate, 1, 1, 1.3, 1);
    % init x_hat with zero bias 
    x_hat = [quat(1,:) 0 0 0];
    
    % get linearizations as functions
    [F,u,h,H] = ekfDerivation();
    
    % uncertainty of init estimate (quat and bias)
    P_k = diag([1e2, 1e2, 1e2, 1e2, 1e4, 1e4, 1e4]);

    % state model cov
    V = diag([.1, .1, .1, .1, 1e-3, 1e-3, 1e-3]);    
    bias = zeros(N, 3);
    for  i=2:N
        % Measurement uncertainty dependent on acceleration norm
        if (abs(norm(acc(i,:))-9.81) < 0.1)
            W = 1e5* eye(6);
        else
            W = 1e7* eye(6);
        end       
        
        % --- prediction using gyro 
        q_gyr = u(1/rate, x_hat(5), x_hat(6), x_hat(7), gyr(i,1), gyr(i,2), gyr(i,3))';        
        x_hat_m = [quaternionMultiply(quat(i-1,:),q_gyr) x_hat(5:7)];
        
        % compute cov of a priori estimate
        F_k = F(1/rate, x_hat(5), x_hat(6), x_hat(7) , gyr(i,1) ,gyr(i,2) ,gyr(i,3),x_hat(1), x_hat(2), x_hat(3), x_hat(4));
        P_pre = F_k*P_k*F_k'+V;
             
        % ---- correction         
        % compute cov and filter gain
        H_k = H(x_hat(1),x_hat(2),x_hat(3),x_hat(4));        
        S_k = W + H_k*P_pre*H_k';
        K_k = P_pre*H_k'/S_k;
        
        % compute reference unit acc vector using prev estimate
        acc_ref = quaternionRotate(quaternionInvert(x_hat(1:4)),[0 0 9.81]);
        acc_ref = acc_ref/norm(acc_ref);
        % project ref mag unit vector into horizontal plane s.t. it only
        % effects the heading
        mag_proj = mag(i,:) - dot(mag(i,:),acc_ref)*acc_ref;
        mag_proj = mag_proj/norm(mag_proj);
        
        % query measurement model
        h_k = h(x_hat(1), x_hat(2), x_hat(3), x_hat(4));
        % compute a posteriro estimate
        x_hat = x_hat_m + (K_k*([acc(i,:) mag_proj] - h_k)')';
        % compute cov of estimate
        P_k = (eye(size(x_hat, 2)) - K_k*H_k)*P_pre*(eye(size(x_hat, 2)) - K_k*H_k)' + K_k*W*K_k';
        
        quat(i,:) = x_hat(1:4);
        bias(i,:) = x_hat(5:7);
    end    
end
