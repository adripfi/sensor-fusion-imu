%% Clear everything
clc; clear; close all;
data = load('data.mat');

%% Flags that specify which variant of the code should be run

opts = struct();
opts.ekf = true; 
opts.realMag = false;

% flags for toggling plots 
opts.plot_raw_data = true; 
opts.plot_vel_pos = true;
opts.save_fig = true;
opts.plot_euler = false; % plot euler angle decomposition 
opts.plot_43 = false; % plot cost function 

% save data as csv for plotting in python
opts.save_csv = false;

%% Write output to a text file
diaryFilename = resultFilename(opts, 'log.txt');
if exist(diaryFilename, 'file') == 2
    delete(diaryFilename);
end
diary(diaryFilename);

%% Load data
accA = data.imu.ua.acc;
gyrA = data.imu.ua.gyr;
magA = data.imu.ua.mag;
if opts.realMag
    magA = data.imu.ua.real_mag;
    magA_fake = data.imu.ua.mag;
end
accC = data.imu.fa.acc;
gyrC = data.imu.fa.gyr;
magC = data.imu.fa.mag;
if opts.realMag
    magC = data.imu.fa.real_mag;
    % magC_fake = data.imu.fa.mag;
end

rate = data.imu.meta.rate;
N = size(accA, 1);
t = 0:(1/rate):((N-1)/rate);

%% Bias compensation
motionStart = 18.5*rate;
motionEnd = 67.5*rate;

motionDuration = motionEnd - motionStart;
% bias before motion start
gyrA_bias_before = repmat(sum(gyrA(1:motionStart,:))/motionStart,[motionStart,1]);
% bias after motion end
gyrA_bias_after = repmat(sum(gyrA(motionEnd:end,:))/(N-motionEnd),[N-motionEnd,1]);
% linear interpolation of bias during motion 
gyrbiasA = [gyrA_bias_before; ...
           [linspace(gyrA_bias_before(1,1), gyrA_bias_after(1,1), motionDuration)' ...
            linspace(gyrA_bias_before(1,2), gyrA_bias_after(1,2), motionDuration)' ...
            linspace(gyrA_bias_before(1,3), gyrA_bias_after(1,3), motionDuration)']; ...
           gyrA_bias_after];

% analogously for IMU C
gyrC_bias_before = repmat(sum(gyrC(1:motionStart,:))/motionStart,[motionStart,1]);
gyrC_bias_after = repmat(sum(gyrC(motionEnd:end,:))/(N-motionEnd),[N-motionEnd,1]);
gyrbiasC = [gyrC_bias_before; ...
           [linspace(gyrC_bias_before(1,1), gyrC_bias_after(1,1), motionDuration)' ...
            linspace(gyrC_bias_before(1,2), gyrC_bias_after(1,2), motionDuration)' ...
            linspace(gyrC_bias_before(1,3), gyrC_bias_after(1,3), motionDuration)']; ...
           gyrC_bias_after];

gyrA_nobias = gyrA - gyrbiasA;
gyrC_nobias = gyrC - gyrbiasC;

figure(100)
subplot(2,1,1)
plotVector(t, rad2deg(gyrbiasA));
title('gyro bias imu A [°/s]'); grid(); legend('show', 'Location','northwest');
subplot(2,1,2)
plotVector(t, rad2deg(gyrbiasC));
title('gyro bias imu C [°/s]'); grid(); legend('show', 'Location','northwest');

%% Plot of raw data
if opts.plot_raw_data
    figure(101)
    ax1 = subplot(3,2,1);
    plotVector(t, accA)
    title('acc A'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset')
    ax2 = subplot(3,2,2);
    plotVector(t, accC)
    title('acc A'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset')
    ax3 = subplot(3,2,3);
    plotVector(t, gyrA_nobias)
    title('gyr A nobias'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset')
    ax4 = subplot(3,2,4);
    plotVector(t, gyrC_nobias)
    title('gyr C nobias'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset')
    ax5 = subplot(3,2,5);
    plotVector(t, magA)
    title('mag A'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset')
    ax6 = subplot(3,2,6);
    plotVector(t, magC)
    title('mag C'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset')
    linkaxes([ax1, ax2, ax3, ax4, ax5, ax6], 'x')
    xlim(t([1,end]))
end

%% Estimate orientation quaternions 

tauAcc = 1;
tauMag = 1;
zeta = 1.3;
accRating = 1;

quatA = estimateQuaternion(accA, gyrA_nobias, magA, rate, tauAcc, tauMag, zeta, accRating);
quatC = estimateQuaternion(accC, gyrC_nobias, magC, rate, tauAcc, tauMag, zeta, accRating);

if opts.realMag  
    % 6D sensor fusion using 6D EKF
    quatA = ekfOrientationEstimation6D(accA, gyrA_nobias, rate);
    quatC = ekfOrientationEstimation6D(accC, gyrC_nobias, rate);
    
    % heading offset found using trial and error with code below
    delta = 6.05;
    % correct heading offset 
    heading_corr = [cos(delta/2) 0 0 sin(delta/2)];
    quatA = quaternionMultiply(heading_corr,quatA);
    
%     % Code used to find heading offset delta
%     rel_quat_A_C = quaternionMultiply(quaternionInvert(quatC(motionEnd,:)), quatA(motionEnd,:));
%     % scalar product of both sensor x axis (longitudinal axis) needs to be
%     % zero 
%     error = dot([1 0 0], quaternionRotate(rel_quat_A_C, [1 0 0]))    
    
end

%% Orientation estimation with EKF

quatA_orig = quatA;
quatC_orig = quatC;
if opts.ekf
    quatA = ekfOrientationEstimation(accA, gyrA, magA, rate);
    quatC = ekfOrientationEstimation(accC, gyrC, magC, rate);  
end

%% Position estimation by integration of accelerations 

% acc of C IMU in ref frame
accC_world = quaternionRotate(quatC,accC);

motion_dur = motionEnd-motionStart+1;

vel_SDI = zeros(N, 3);
grav = [0 0 9.81];
% remove gravity from acc readings and integrate over motion period
vel_SDI(motionStart:motionEnd,:) = cumsum((accC_world(motionStart:motionEnd, :) - grav)/rate);

% estimate velocity drift at motion end (resting position)
vel_drift = ones(motion_dur,1)*vel_SDI(motionEnd,:)*rate/motion_dur;

vel_SDI_nodrift = zeros(N, 3); 
% correct for drift 
vel_SDI_nodrift(motionStart:motionEnd, :) = cumsum((accC_world(motionStart:motionEnd, :)- vel_drift- grav)/rate);
% integrate velocity to get position estimates
pos_SDI=cumsum(vel_SDI_nodrift/rate);    


if opts.plot_vel_pos
    figure(102)
    ax1 = subplot(3,1,1);
    plotVector(t, vel_SDI)
    title('vel\_SDI'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset');
    ax2 = subplot(3,1,2);
    plotVector(t, vel_SDI)
    title('vel\_SDI\_nodrift'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset');
    ax3 = subplot(3,1,3);
    plotVector(t, pos_SDI)
    title('pos\_SDI'); grid(); legend('show', 'Location','northwest'); zoom(gcf,'reset');

    figure(103)
    plot3(pos_SDI(:,1), pos_SDI(:,2), pos_SDI(:,3));
    title('3D plot of pos\_SDI'); axis('equal'); grid();

    figure(1)
    plot(pos_SDI(:,1), pos_SDI(:,2));
    title('x-y-projection of pos\_SDI'); axis('equal');
end

%% Position estimation using the orientation of both segments 
l_ua = 0.4;
l_fa = 0.4;

pos_kin = zeros(N,3);
% each segement has a length of 0.4m in local x direction 
init_pos = [0.4 0 0];

for i=1:N
    % compute elbow position by rotating the forearm 
    pos_elb = quaternionRotate(quatC(i,:),init_pos);
    % compute wrist position by rotating the upperarm and "attaching" it to
    % the forearm
    pos_kin(i,:) = pos_elb + quaternionRotate(quatA(i,:),init_pos);
end

figure(104)
plotVector(t, pos_kin)
title('pos\_joint over time'); legend('show', 'Location','northwest'); grid();

figure(105)
plot3(pos_kin(:,1), pos_kin(:,2), pos_kin(:,3))
axis('equal');
title('3D plot of pos\_joint'); legend('show', 'Location','northwest'); grid();

figure(2)
plot(pos_kin(:,1), pos_kin(:,2));
title('x-y-projection of pos\_joint'); axis('equal');

%% Use a threshold to separate the segments
pos_thresholded = pos_kin;

% treshold for z value 
th_up = 0.022; 
th_down = 0.008;

if opts.realMag
    th_up = 0.08;    
end

for i=1:N
    % set the values that shouldn't be plotted to NaN
    if pos_thresholded(i,3) > th_up || pos_thresholded(i,3) < th_down
        pos_thresholded(i,:) = NaN;
    end
end

figure(2); hold all;
lh=plot(pos_thresholded(:,1), pos_thresholded(:,2), 'k');
lh.Color=[0,0,0,0.5];
lh.LineWidth=2;

%% Rotational degrees of freedom of the joint

% relative orientation between A and C frame
quatRelCA = quaternionMultiply(quaternionInvert(quatA),quatC);
[w, x, y, z,] = deal(quatRelCA(:,1), quatRelCA(:,2), quatRelCA(:,3), quatRelCA(:,4));
% calculate euler angles using x-z'-y'' convention i.e. alpha, gamma, beta
% here the carrying angle corresponds to beta, which does only hold if the
% arm shown in the picture is already rotated around x 
alpha=atan2(2.*(x.*w+y.*z),(w.^2-x.^2+y.^2-z.^2));
beta=asin(2.*(z.*w-x.*y));
gamma=atan2(2.*(x.*z+y.*w),(w.^2+x.^2-y.^2-z.^2));

% unwrap phase angle and convert from rad to deg
alpha = rad2deg(unwrap(alpha));
beta = rad2deg(unwrap(beta));
gamma = rad2deg(unwrap(gamma));

% plot 
if opts.plot_euler
    figure(402);
    title('Euler angle decomposition')
    subplot(3,1,1);
    plot(alpha);
    title('\alpha');
    subplot(3,1,2);
    plot(beta);
    title('\beta');
    subplot(3,1,3);
    plot(gamma);
    title('\gamma');
end

%% Joint axis identification

% s2 is upper arm i.e. frame A and s1 is forearm i.e. C frame
% j2 is jab and j1 is jbc
global quatC_glob
quatC_glob = quatC;

% relative gyroscope readings in world frame 
gyrA_world = quaternionRotate(quatA, gyrA_nobias);
gyrC_world = quaternionRotate(quatC, gyrC_nobias);
global om_rel
om_rel = gyrC_world - gyrA_world;

% x axis (joint j_ab) in world frame
global j2_world
j2_world = quaternionRotate(quatA, [1 0 0]);

% solve lsq problem
options = optimoptions(@lsqnonlin,'Algorithm', 'levenberg-marquardt');
% options.Display = 'iter';
[x,resnorm] = lsqnonlin(@cost_func_43, [0 0], [0 0], [pi 2*pi], options);

% convert result from spherical to cartesion coordinates
disp('4.3')
j_bc_C = [sin(x(1))*cos(x(2)) sin(x(1))*sin(x(2)) cos(x(1))]

% plot cost function
if opts.plot_43
    figure(403);
    f1 = @(x,z) cost_func_43([acos(z) atan(sqrt(1 - x^2 - z^2)/x)])'*cost_func_43([acos(z) atan(sqrt(1 - x^2 - z^2)/x)]);
    fsurf(f1,[-1 1 -1 1])
    title('Cost function over x and z coordinates of elbow joint axis')
    xlabel('x') 
    ylabel('z') 
    zlabel('cost') 
end

%% Joint distance identification

% declare global variables for cost function 
global gyrA_glob
global gyrC_glob
global accA_glob
global accC_glob
gyrA_glob = gyrA_nobias;
gyrC_glob = gyrC_nobias;
accA_glob = accA;
accC_glob = accC;

% gyroscope time derivates
global dgyrA
global dgyrC
dgyrA = [0 0 0 ; (gyrA_nobias(2:N,:) - gyrA_nobias(1:N-1,:)).*rate];
dgyrC = [0 0 0 ; (gyrC_nobias(2:N,:) - gyrC_nobias(1:N-1,:)).*rate];

% solve minimazation problem
options = optimoptions(@lsqnonlin,'Algorithm', 'levenberg-marquardt');
% options.Display = 'iter';
[x,resnorm] = lsqnonlin(@cost_func_44, [0 0 0 0 0 0], [], [], options);

% results
o_1 = x(1:3);
o_2 = x(4:6);
disp('4.4')
info = sprintf('||o_1||=%s, ||o_2||=%s',norm(o_1), norm(o_2));
disp(info)

% save results as csv for plotting in python
if opts.save_csv
    if opts.realMag
        writematrix(beta, 'carry_angle_6D_ekf.csv')
        writematrix(pos_thresholded(:,1:2), 'writing_6D_ekf.csv')
    elseif opts.ekf
        writematrix(beta, 'carry_angle_9D_ekf.csv')
        writematrix(pos_thresholded(:,1:2), 'writing_9D_ekf.csv')
    else
        writematrix(beta, 'carry_angle_9D.csv')
        writematrix(pos_thresholded(:,1:2), 'writing_9D.csv')
    end
end


%% Save result and plots and close log file
if opts.save_fig
    save(resultFilename(opts, 'results.mat'), 'opts', 'gyrbiasA', 'gyrbiasC', 'gyrA_nobias', 'gyrC_nobias', 'tauAcc', 'tauMag', 'zeta', 'accRating', 'quatA', 'quatC', 'quatA_orig', 'quatC_orig', 'vel_SDI', 'vel_SDI_nodrift', 'pos_SDI', 'pos_kin', 'pos_thresholded', 'th_up', 'th_down');

    for f=findobj('Type', 'figure')'
        figFilename = resultFilename(opts, ['figure_' num2str(f.Number) '.pdf']);
        fprintf('saving figure %s\n', figFilename);
        saveas(f, figFilename);
    end
end
diary off;
