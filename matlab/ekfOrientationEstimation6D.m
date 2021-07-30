function [quat] = ekfOrientationEstimation6D(acc, gyr, rate)
    N = size(acc, 1);
    
    %% <task 3.6>
    quat = zeros(N, 4);
    quat(:,1) = 1;
        
    % inital estimate
    q_k = quaternionFromAcc(acc(1,:));
    u_k = q_k;
    
    % cov matrices
    P_k = 10*eye(4);
    V = 0.1*eye(4);
    W = 1e12* eye(3);
    
    for  i=1:N
        F = [[u_k(1) -u_k(2) -u_k(3) -u_k(4)];
             [u_k(2)  u_k(1)  u_k(4) -u_k(3)];
             [u_k(3) -u_k(4)  u_k(1)  u_k(2)];
             [u_k(4)  u_k(3) -u_k(2)  u_k(1)];
            ];
        
        H_k = 2*9.81*[[-q_k(3)  q_k(4) -q_k(1) q_k(2)];
                      [ q_k(2)  q_k(1)  q_k(4) q_k(3)];
                      [ q_k(1) -q_k(2) -q_k(3) q_k(4)]];
                              
        
        P_k_m = F * P_k * F' + V;
        
        K = P_k_m * H_k' * inv(W + H_k * P_k_m * H_k');
        
        % prediction 
        u_k = quaternionFromGyr(gyr(i,:), rate);
        q_k_m = quaternionMultiply(q_k,u_k);
        y_k =  [acc(i,:)];
        
        % correction 
        h_k = quaternionRotate(q_k_m,[0 0 9.81]);
        q_k = q_k_m + (K*(y_k - h_k)')'; 
        quat(i,:) = q_k;
        
        P_k = (eye(4) - K * H_k)* P_k_m * (eye(4) - K * H_k)' + K * W * K';      
    end 
    %% </task 3.6>
end
