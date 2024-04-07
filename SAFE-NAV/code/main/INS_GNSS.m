%% pre-code
clc
% clear all

%% load data

%zupt = false;
load('imu1.mat');
load('ref.mat');
load('gnss.mat');
ti=imu1.t;
tg=gnss.t;
fb = imu1.fb;
wb = imu1.wb;
zupt = false;
kf.L=0;
kf.J=0;
kf.V=zeros(6,1);
kf.mat=zeros(6,6);
kf.mean=zeros(6,6);
kf.ROR=[];
kf.TT=[];

%% initialization

I = eye(3);
O = zeros(3);

R2D = (180/pi);     
NOS = length(ti);
NOS_g = length(tg);
kf.VVV = zeros(1,NOS_g);
kf.VVVCCC = zeros(1,NOS_g);

ini_align = [0 0 0];
ini_vel = [0 0 0];
ini_lat = gnss.lat(1);
ini_lon = gnss.lon(1);
ini_h = gnss.h(1);

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

%% pre-processing

%xx    = zeros (NOS,1);
%yy    = zeros (NOS,1);
%zz    = zeros (NOS,1);
%XYZ = zeros (NOS,3);
%XYZ(1,:) = [0 0 0];
%E_est = [0.01 0.01 0.01];
%E_meas = [0.01 0.01 0.01]; 
%EST = [ini_lat  ini_lon  ini_h];
gb_dyn = imu1.gb_dyn';
ab_dyn = imu1.ab_dyn';
kf.xi = [ zeros(1,9), imu1.gb_dyn, imu1.ab_dyn ]';
kf.Pi = diag([imu1.ini_align_err, gnss.stdv, gnss.std, imu1.gb_dyn, imu1.ab_dyn].^2);
kf.R  = diag([gnss.stdv, gnss.stdm].^2);
kf.Q  = diag([imu1.arw, imu1.vrw, imu1.gb_psd, imu1.ab_psd].^2);
fb(1,:) = (fb(1,:) + imu1.ab_dyn );
fn = (DCMbn * fb(1,:)');
upd = [gnss.vel(1,:) gnss.lat(1) gnss.h(1) fn'];
[kf.F, kf.G] = F_update(upd, DCMbn, imu1); % A and B of the system 
[R_N,R_E] = earth (gnss.lat(1));
Tpr = diag([(R_N + gnss.h(1)), (R_E + gnss.h(1)) * cos(gnss.lat(1)), -1]);  
kf.H = [ O I O O O ;
    O O Tpr O O ; ];
kf.R = diag([gnss.stdv gnss.stdm]).^2;
kf.z = [ gnss.stdv, gnss.stdm ]';


kf = kf_update( kf );

xi = zeros(NOS_g, 15);        
xp = zeros(NOS_g, 15);        
z = zeros(NOS_g, 6);          
v = zeros(NOS_g, 6);          
b  = zeros(NOS_g, 6);         
A  = zeros(NOS_g, 225);       
Pi = zeros(NOS_g, 225);       
Pp = zeros(NOS_g, 225);       
K  = zeros(NOS_g, 90);       
S  = zeros(NOS_g, 36);       
xp(1,:) = kf.xp';
Pp(1,:) = reshape(kf.Pp, 1, 225);
b(1,:)  = [imu1.gb_sta, imu1.ab_sta];


%%
for i = 2:NOS 
    
    fb(i,:) = (fb(i,:) + ab_dyn' );
    wb(i,:) = (wb(1,:) + gb_dyn' );
    wb_corrected = (imu1.wb(i,:)' + gb_dyn );
    fb_corrected = (imu1.fb(i,:)' + ab_dyn );
    
    dti = ti(i) - ti(i-1);
    w_ie_n = Compu_w_ie_n (lat(i-1));
    w_en_n = Compu_w_en_n(lat(i-1), vel(i-1,:), h(i-1));
    [w_nb_b] = Compu_w_nb_b (w_ie_n,w_en_n,wb(i,:),DCMnb);
    [q_update] = quaternion_update (w_nb_b,q,dti);
    [DCMbn] = quaternion2DCMbn (q);
    [euler] = DCMbn2Attitude (DCMbn);
    roll(i) = euler(1);
    pitch(i)= euler(2);
    yaw(i)  = euler(3);
    q=q_update;
    g_n = gravityINS(lat(i-1), h(i-1));
    fn = (DCMbn * fb(i,:)');
    [f_e_n] = acceleration_update(fn,w_ie_n,w_en_n,vel(i-1,:),g_n');
    [vel_n] = velocity_update(f_e_n,vel(i-1,:),dti);
    vel (i,:) = vel_n;
    %[XYZ_n] = XYZ_update(vel_n,XYZ(i-1,:),dt);
    %XYZ (i,:) = XYZ_n;
    %xx(i) = XYZ (i,1);
    %yy(i) = XYZ (i,2);
    %zz(i) = XYZ (i,3);
    pos = position_update([lat(i-1) lon(i-1) h(i-1)], vel(i,:), dti);
    lat(i) = pos(1);
    lon(i) = pos(2);
    h(i)   = pos(3);
    %MEA = [lat(i) lon(i) h(i)];
    %[EST_n,E_est_n] = Kalman_INS (E_est,E_meas,EST,MEA);
    %E_est=E_est_n;
    %EST = EST_n;
    %lat(i) = EST_n(1);
    %lon(i) = EST_n(2);
    %h(i) = EST_n(3);
%%    
  idz = floor( gnss.zupt_win / dti ); % Index to set ZUPT window time
    
    if ( i > idz )
        
        vel_m = mean (vel(i-idz:i , :));
        
        if (abs(vel_m) <= gnss.zupt_th)
            
            roll_e(i) = mean (roll_e(i-idz:i , :));
            pitch_e(i)= mean (pitch_e(i-idz:i , :));
            yaw_e(i)  = mean (yaw_e(i-idz:i , :));
            
            lat_e(i) = mean (lat_e(i-idz:i , :));
            lon_e(i) = mean (lon_e(i-idz:i , :));
            h_e(i)   = mean (h_e(i-idz:i , :));
            
            zupt = true;

        end
    end
    
    %% KALMAN FILTER UPDATE
    
    % Check if there is new GNSS data to process at current INS time
    gdx =  find (gnss.t >= (imu1.t(i) - gnss.eps) & gnss.t < (imu1.t(i) + gnss.eps));
    
    if ( ~isempty(gdx) && gdx > 1)
        
        %% INNOVATIONS
        [R_N,R_E] = earth(lat(i));
        Tpr = diag([(R_N + h(i)), (R_E + h(i)) * cos(lat(i)), -1]);  % radians-to-meters
        
        % Innovations for position with lever arm correction
        zp = Tpr * ([lat(i); lon(i); h(i);] - [gnss.lat(gdx); gnss.lon(gdx); gnss.h(gdx);]) ...
            + (DCMbn * gnss.larm);
        
        % Innovations for velocity with lever arm correction
        zv = (vel(i,:) - gnss.vel(gdx,:) - ((w_ie_n + w_en_n) .* (DCMbn * gnss.larm))' ...
            + (DCMbn * skewm(wb_corrected) * gnss.larm )' )';
        
        %% KALMAN FILTER
        
        % GNSS sampling interval
        dtg = gnss.t(gdx) - gnss.t(gdx-1);
        
        % Vector to update matrix F
        upd = [vel(i,:) lat(i) h(i) fn'];
        
        % Update matrices F and G
        [kf.F, kf.G] = F_update(upd, DCMbn, imu1);
        
        % Update matrix H
        if(zupt == false)
            kf.H = [ O I O O O ;
                     O O Tpr O O ; ];
            kf.R = diag([gnss.stdv gnss.stdm]).^2;
            kf.z = [ zv' zp' ]';
        else
            kf.H = [ O I O O O ; ];
            kf.R = diag([gnss.stdv]).^2;
            kf.z = zv;
        end
        
        % Execute the extended Kalman filter
        kf.xp(1:9) = 0;              % states 1:9 are forced to be zero (error-state approach)
        kf = kalmanINS(kf, dtg,i,FUZZ,ti(i));
        
        %% INS/GNSS CORRECTIONS
        
        antm = [0 q_update(3) -q_update(2); -q_update(3) 0 q_update(1); q_update(2) -q_update(1) 0];
        q = q_update + 0.5 .* [q_update(4)*eye(3) + antm; -1.*[q_update(1) q_update(2) q_update(3)]] * kf.xp(1:3);
        q = q / norm(q);       % Brute-force normalization
        
        % DCM correction
        DCMbn = qua2dcm(q);
        
        % Attitude corrections
        roll(i)  = roll(i)  - kf.xp(1);
        pitch(i) = pitch(i) - kf.xp(2);
        yaw(i)   = yaw(i)   - kf.xp(3);
        
        % Velocity corrections
        vel (i,1) = vel (i,1) - kf.xp(4);
        vel (i,2) = vel (i,2) - kf.xp(5);
        vel (i,3) = vel (i,3) - kf.xp(6);
        
        % Position corrections
        lat(i) = lat(i) - (kf.xp(7));
        lon(i) = lon(i) - (kf.xp(8));
        h(i)   = h(i)   - kf.xp(9);
        
        % Biases corrections
        gb_dyn   = kf.xp(10:12);
        ab_dyn   = kf.xp(13:15);
        
        % Matrices for later Kalman filter performance analysis
        xi(gdx,:) = kf.xi';
        xp(gdx,:) = kf.xp';
        b(gdx,:) = [gb_dyn', ab_dyn'];
        A(gdx,:)  = reshape(kf.A, 1, 225);
        Pi(gdx,:) = reshape(kf.Pi, 1, 225);
        Pp(gdx,:) = reshape(kf.Pp, 1, 225);        
              
        if(zupt == false)
            v(gdx,:)  = kf.v';
            K(gdx,:)  = reshape(kf.K, 1, 90);
            S(gdx,:)  = reshape(kf.S, 1, 36);
        else
            zupt = false;
            v(gdx,:)  = [ kf.v' 0 0 0 ]';
            K(gdx,1:45)  = reshape(kf.K, 1, 45);
            S(gdx,1:9)  = reshape(kf.S, 1, 9);
        end
    end  
    
end
  
%% trajectory

     figure(1);  
    
     %subplot(321) %estimated and referance
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
%     
%     subplot(322) %error in lat
     figure(2)   
    err_lon= ref.lon.*R2D - lon.*R2D;
     
     sum(err_lon.^2)
     
     plot(ti , err_lon, 'b')
     xlabel('Time [sec]')
     ylabel('Longitude error [deg]')
     title ('error in Longitude')
     grid
     hold on
%     
%     subplot(323) %error in lon
figure(3)
    err_lat= ref.lat.*R2D - lat.*R2D;
%     
     sum(err_lat.^2)
     plot(ti , err_lat, 'b')
     xlabel('Time [sec]')
     ylabel('Latitude error [deg]')
     title ('error in Latitude')
     grid
     hold on
%  
%     
%     
%     subplot(324) %error in h
figure(4)
    err_h= ref.h.*R2D - h.*R2D;
    sum(err_h.^2)
     plot(ti , err_h, 'b')
     xlabel('Time [sec]')
     ylabel('Altitude error [m]')
     title ('error in Altitude')
    grid
    hold on
%

figure(5)
    err_roll= ref.roll - roll;
     sum(err_roll.^2)
     plot(ti , err_roll, 'b')
     xlabel('Time [sec]')
     ylabel('Roll angle error [degree]')
     title ('error in the Roll angle')
     grid
     hold on

     figure(6)
     err_pitch= ref.pitch - pitch;
     sum(err_pitch.^2)
     plot(ti , err_pitch, 'b')
     xlabel('Time [sec]')
     ylabel('Pitch angle error [degree]')
     title ('error in the Pitch angle')
     grid
     hold on
     
     figure(7)
     err_yaw= ref.yaw - yaw;
     sum(err_yaw.^2)
     plot(ti , err_yaw, 'b')
     xlabel('Time [sec]')
     ylabel('Yaw angle error [degree]')
     title ('error in the Yaw angle')
     grid
     hold on
     
     figure(8)
     err_vn= ref.vel(:,1) - vel(:,1);
     sum(err_vn.^2)
     plot(ti , err_vn, 'b')
     xlabel('Time [sec]')
     ylabel('North Velocity error [m/s]')
     title ('error in the North Velocity')
     grid
     hold on
     
     figure(9)
     err_ve= ref.vel(:,2) - vel(:,2);
     sum(err_ve.^2)
     plot(ti , err_ve, 'b')
     xlabel('Time [sec]')
     ylabel('East Velocity error [m/s]')
     title ('error in the East Velocity')
     grid
     hold on
     
     figure(10)
     err_vd= ref.vel(:,3) - vel(:,3);
     sum(err_vd.^2)
     plot(ti , err_vd, 'b')
     xlabel('Time [sec]')
     ylabel('Down Velocity error [m/s]')
     title ('error in the Down Velocity')
     grid
     hold on
     
%     subplot(325) %ref alone
%     plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h, '--k')
%     xlabel('ref lon [deg]')
%     ylabel('ref lat [deg]')
%     zlabel('ref heading [m]')
%     title ('ref lon vs ref lat')
%     grid
%     
%     subplot(326) %estimated alone
%     plot3(lon.*R2D, lat.*R2D, h, 'b')
%     xlabel('lon [deg]')
%     ylabel('lat [deg]')
%     zlabel('heading [m]')
%     title ('lon vs lat')
%     grid
%     
    
    %figure(2)
    %hold on
    %HHHH=1:2388;
    %plot(HHHH,kf.ROR)
    %hold on
    %plot(ti , err_lat,'g')
    %xlabel('Time [sec]')
    %ylabel('lat error [deg]')
    %title ('error in lat')
    %grid
    %hold on
    
    %figure(3)
    %hold on
    %plot(ti , err_lat,'g')
    %xlabel('Time [sec]')
    %ylabel('lat error [deg]')
    %title ('error in lat')
    %grid
    %hold on
    
    %figure(4)
    %hold on
    %plot(ti , err_lat,'b')
    %xlabel('Time [sec]')
    %ylabel('lat error [deg]')
    %title ('error in lat')
    %grid
    %hold on
    
    %figure(5)
    %hold on
    %yyaxis left
    %plot(ti , err_lat,'y')
    %xlabel('Time [sec]')
    %ylabel('lat error [deg]')
    %title ('error in lat')
    %grid
    %hold on
    
%     figure(5)
%     hold on
%     yyaxis right
%     plot(kf.TT , kf.ROR,'c')
%     ylim([-180 100])
%     xlabel('Time [sec]')
%     ylabel('ROR')
%     title ('ROR vs time')
%     grid on
%     hold on