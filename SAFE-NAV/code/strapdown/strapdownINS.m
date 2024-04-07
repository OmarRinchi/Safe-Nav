%% All prameters

% IMU date:
% t: time vector
% fb: measured acceleration in b-frame
% wb: measured turn rate in b-frame
% ini_align: initial roll, pitch and yaw at time t(1)

% initialization date:
% NOS: the length of samples used in simulation
% roll: roll angle
% pitch: pitch angle
% yaw: yaw angle
% DCMnb: direct cosine matrix that transforms from n-frame to b-frame
% DCMbn: direct cosine matrix that transforms from b-frame to n-frame
% qua: quatanion vector
% vel: velocity in the n-frame
% ini_vel: initial velocity
% lat: latitude
% lon: longitude
% h: heading
% ini_lat: initial latitude
% ini_lon: initial longitude
% ini_h: initial heading
% i: loop counter
% dt: sampling time
% omega_ie_n: earth turn rate in n-frame
% omega_en_n: navigation frame turn rate with respect to the earth in the
% n-frame
% att_mode: attitude mode string.
%      'quaternion': attitude updated in quaternion format. Default value.
%      'dcm': attitude updated in Direct Cosine Matrix format.
% qua_n: updated quatanion vector
% euler: vector that contains updated roll, pitch and yaw angles
% gn: gravity vector in the n-frame
% fn: measured acceleration in the f-frame
% vel_n updated velocity vector
% pos: updated position vector

%% pre-code
clc
clear all

%% load data

load('imu1.mat');
load('ref.mat');
t=imu1.t;
fb = imu1.fb;
wb = imu1.wb;


ini_align = imu1.ini_align;
ini_vel = [0.105527057243551,0.00375455398352848,-0.0545998944220750];
ini_lat = [-0.573003956064926];
ini_lon = [-1.20065977516480];
ini_h = [690.923205585682];

%% initialization

R2D = (180/pi);     % radians to degrees
NOS = length(t);
roll = zeros(NOS,1);
pitch = zeros(NOS,1);
yaw = zeros(NOS,1);
roll(1) = ini_align(1);
pitch(1) = ini_align(2);
yaw(1) = ini_align(3);
DCMnb = attitude2DCM(roll(1), pitch(1), yaw(1));
DCMbn = DCMnb';
q   = attitude2quaternion (roll(1), pitch(1), yaw(1));
vel   = zeros (NOS, 3);
vel(1,:) = ini_vel;
lat    = zeros (NOS,1);
lon    = zeros (NOS,1);
h      = zeros (NOS, 1);
lat(1) = ini_lat;
lon(1) = ini_lon;
h(1) = ini_h;
x    = zeros (NOS,1);
y    = zeros (NOS,1);
z    = zeros (NOS,1);
XYZ = zeros (NOS,3);
XYZ(1,:) = [0 0 0];
 %
 h=ref.h;
 %
EST = zeros(NOS,3);
E_est = [0.01 0.01 0];
E_meas = [0.01 0.01 0];
EST(1,:) = [ini_lat  ini_lon  h(1)];
fb=fb*0;
wb=wb*0;
%%
for i = 2:NOS

    dt = t(i) - t(i-1);
    w_ie_n = Compu_w_ie_n (lat(i-1));
    w_en_n = Compu_w_en_n(lat(i-1), vel(i-1,:), h(i-1));
    [w_nb_b] = Compu_w_nb_b (w_ie_n,w_en_n,wb(i,:),DCMnb);
    [q_update] = quaternion_update (w_nb_b,q,dt);
    %q=q_update;
    [DCMbn] = quaternion2DCMbn (q);
    [euler] = DCMbn2Attitude (DCMbn);
    roll(i) = euler(1);
    pitch(i)= euler(2);
    yaw(i)  = euler(3);
    q=q_update;
    g_n = gravityINS(lat(i-1), h(i-1));
    fn = (DCMbn * fb(i,:)');
    [f_e_n] = acceleration_update(fn,w_ie_n,w_en_n,vel(i-1,:),g_n');
    [vel_n] = velocity_update(f_e_n,vel(i-1,:),dt);
    vel (i,:) = vel_n;
    [XYZ_n] = XYZ_update(vel_n,XYZ(i-1,:),dt);
    XYZ (i,:) = XYZ_n;
    x(i) = XYZ (i,1);
    y(i) = XYZ (i,2);
    z(i) = XYZ (i,3);
    pos = position_update([lat(i-1) lon(i-1) h(i-1)], vel(i,:), dt);
    lat(i) = pos(1);
    lon(i) = pos(2);
    %h(i)   = pos(3);
    MEA = [lat(i) lon(i) h(i)];
    [EST_n,E_est_n] = Kalman_INS (E_est,E_meas,EST(i-1),MEA);
    E_est=E_est_n;
    EST(i,:) = EST_n;
    %lat(i) = EST_n(1);
    %lon(i) = EST_n(2);
    %h(i) = EST_n(3);
end

%% trajectory

    figure(1);  
    subplot(321) %estimated and referance
    plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h, '--k')
    hold on
    plot3(lon.*R2D, lat.*R2D, h, 'b')
    plot3(ref.lon(1).*R2D, ref.lat(1).*R2D, ref.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
    axis tight
    title('TRAJECTORIES')
    xlabel('Longitude [deg]')
    ylabel('Latitude [deg]')
    zlabel('Altitude [m]')
    view(-25,35)
    legend('TRUE', 'IMU1')
    grid
    
    subplot(322) %error in lat
    err_lon= ref.lon .*R2D - lon.*R2D;
    plot(t , err_lon)
    xlabel('Time [sec]')
    ylabel('lon error [deg]')
    title ('error in lon')
    grid
    
    subplot(323) %error in lon
    err_lat= ref.lat.*R2D - lat.*R2D;
    plot(t , err_lat)
    xlabel('Time [sec]')
    ylabel('lat error [deg]')
    title ('error in lat')
    grid
    
    subplot(324) %error in h
    err_h= ref.h.*R2D - h.*R2D;
    plot(t , err_h)
    xlabel('Time [sec]')
    ylabel('h error [m]')
    title ('error in h')
    grid
    
    subplot(325) %ref alone
    plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h, '--k')
    xlabel('ref lon [deg]')
    ylabel('ref lat [deg]')
    zlabel('ref heading [m]')
    title ('ref lon vs ref lat')
    grid
    
    subplot(326) %estimated alone
    plot3(lon.*R2D, lat.*R2D, h, 'b')
    xlabel('lon [deg]')
    ylabel('lat [deg]')
    zlabel('heading [m]')
    title ('lon vs lat')
    grid