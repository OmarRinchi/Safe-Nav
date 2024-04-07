function  [OUT,ref,ti] = Main_EKF(OPT)

%% pre-code
clc
%clear all
warning off
loop = waitbar(0,'Pre-processing, please wait...');

%% load data
if(OPT==1)
load('MM.mat');
load('STAT');
end
if(OPT==2)
load('SS.mat');
load('STAT');
end
if(OPT==3)
load('TT.mat');
load('STAT');
VAR=struct2cell(STAT);
STAT.arw=VAR{1}; 
STAT.arr=VAR{2}; 
STAT.vrw=VAR{3}; 
STAT.vrrw=VAR{4}; 
STAT.gb_sta=VAR{5}; 
STAT.ab_sta=VAR{6}; 
STAT.gb_dyn=VAR{7}; 
STAT.ab_dyn=VAR{8}; 
STAT.gb_corr=VAR{9};
STAT.ab_corr=VAR{10}; 
STAT.a_std=VAR{11}; 
STAT.g_std=VAR{12}; 
STAT.gb_psd=VAR{13}; 
STAT.ab_psd=VAR{14}; 
STAT.ini_align_err=VAR{15}; 
STAT.ini_align=VAR{16}; 
STAT.stdm=VAR{17}; 
STAT.stdv=VAR{18}; 
STAT.larm=VAR{19}; 
STAT.zupt_th=VAR{20}; 
STAT.zupt_win=VAR{21}; 
STAT.eps=VAR{22}; 
STAT.std=VAR{23};
end
FUZZ = 0;
%TT = synchronize(Acceleration,AngularVelocity,Position,'first','linear','SampleRate',0.1);
TT = synchronize(Acceleration,AngularVelocity,Position,'first','linear');
AccelerationX = TT(:,[1 2 3]);
AngularVelocityX = TT(:,[4 5 6]);
PositionX = TT(:,[7 8 9]);

%% define data: construct acceleration varaible, angular velocity variable and position variable
fb = table2array(AccelerationX);
wb = table2array(AngularVelocityX);
PositionXX = table2array(Position);
Positionref = table2array(PositionX);
lat = PositionXX(:,1) * pi/180;
lon = PositionXX(:,2) * pi/180; 
h = PositionXX(:,3); 
 
%% re-construct the structures
GPS.lat = lat;
GPS.lon = lon;
GPS.h = h;
GPS.vel = PositionXX(:,4);
ref.lat = Positionref(:,1) * pi/180; ;
ref.lon = Positionref(:,2) * pi/180; ;
ref.h = Positionref(:,3);
close (loop)
loop = waitbar(0,'creating time vector, please wait...');

%% time: construct the time variable in the unit of ms
t=Acceleration.Timestamp;
DTN = length(t);
ti = zeros(DTN,1);
for j = 2 : DTN
        if(mod(j,1000)==0)
        waitbar(j/DTN);
    end
    ti(j)= milliseconds(t(j) - t(j-1))/1000 + ti(j-1);
end
t=Position.Timestamp;
DTN = length(t);
tg = zeros(DTN,1);
for k = 2 : DTN
    if(mod(k,100)==0)
        waitbar(k/DTN);
    end
    tg(k)= milliseconds(t(k) - t(k-1))/1000 + tg(k-1);
end
close(loop)

%% initialization
R2D = (180/pi);
D2R = (pi/180);
NOS = length(ti);
NOS_g = length(tg);
KALMAN.L=0;
KALMAN.V=zeros(6,1);
KALMAN.mat=zeros(6,6);
KALMAN.mean=zeros(6,6);
KALMAN.ROR=[];
KALMAN.TT=[];
I = eye(3);
O = zeros(3);
ini_align = STAT.ini_align;
ini_vel = [0 0 0];
ini_lat = GPS.lat(1);
ini_lon = GPS.lon(1);
ini_h = GPS.h(1);
roll = zeros(NOS,1);
pitch = zeros(NOS,1);
yaw = zeros(NOS,1);
roll(1) = ini_align(1);
pitch(1) = ini_align(2);
yaw(1) = ini_align(3);
DCMnb = attitude2DCM(roll(1), pitch(1), yaw(1)); %initial DCM matrix
DCMbn = DCMnb';
q   = attitude2quaternion (roll(1), pitch(1), yaw(1)); %initial quaternion
vel   = zeros (NOS, 3);
vel(1,:) = ini_vel;
lat    = zeros (NOS,1);
lon    = zeros (NOS,1);
h      = zeros (NOS, 1);
lat(1) = ini_lat;
lon(1) = ini_lon;
h(1) = ini_h;

%% pre-processing
gb_dyn = STAT.gb_dyn';
ab_dyn = STAT.ab_dyn';
KALMAN.xi = [ zeros(1,9), STAT.gb_dyn, STAT.ab_dyn ]';
KALMAN.Pi = diag([STAT.ini_align_err, STAT.stdv, STAT.std, STAT.gb_dyn, STAT.ab_dyn].^2);
KALMAN.R  = diag([STAT.stdv, STAT.stdm].^2);
KALMAN.Q  = diag([STAT.arw, STAT.vrw, STAT.gb_psd, STAT.ab_psd].^2);
fb(1,:) = (fb(1,:) + STAT.ab_dyn );
fn = (DCMbn * fb(1,:)');
UPDATE = [[0 0 0] GPS.lat(1) GPS.h(1) fn'];
[KALMAN.F, KALMAN.G] = model(UPDATE, DCMbn, STAT.ab_corr, STAT.gb_corr);
[R_N,R_E] = earth (GPS.lat(1));
Tpr = diag([(R_N + GPS.h(1)), (R_E + GPS.h(1)) * cos(GPS.lat(1)), -1]);  
KALMAN.H = [ O I O O O ; O O Tpr O O ; ];
KALMAN.R = diag([STAT.stdv STAT.stdm]).^2;
KALMAN.z = [ STAT.stdv, STAT.stdm ]';
KALMAN.S = (KALMAN.R + KALMAN.H * KALMAN.Pi * KALMAN.H');
KALMAN.v =  KALMAN.z - KALMAN.H * KALMAN.xi;
KALMAN.K = (KALMAN.Pi * KALMAN.H') * (KALMAN.S)^(-1) ;				
KALMAN.xp = KALMAN.xi + KALMAN.K * KALMAN.v; 
KALMAN.Pp = KALMAN.Pi - KALMAN.K * KALMAN.S *  KALMAN.K';            
KALMAN.Pp =  0.5 .* (KALMAN.Pp + KALMAN.Pp');              
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
xp(1,:) = KALMAN.xp';
Pp(1,:) = reshape(KALMAN.Pp, 1, 225);
b(1,:)  = [STAT.gb_sta, STAT.ab_sta];

%% strapdown
loop = waitbar(0,'Processing, Please wait...');
for i = 2:NOS 
    if(mod(i,1000)==0)
    waitbar(i/NOS);
    end
    fb(i,:) = (fb(i,:) + ab_dyn' );
    wb(i,:) = (wb(1,:) + gb_dyn' );
    wb_corrected = (wb(i,:)' + gb_dyn );
    fb_corrected = (fb(i,:)' + ab_dyn );
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
    pos = position_update([lat(i-1) lon(i-1) h(i-1)], vel(i,:), dti);
    lat(i) = pos(1);
    lon(i) = pos(2);
    h(i)   = pos(3);
%% ZERO Velocity   
  [lat,lon,h,roll,pitch,yaw] = ZeroVelocity(STAT,dti,i,vel,lat,lon,h,roll,pitch,yaw);
    
%% KALMAN FILTER
 gdx =  find (tg >= (ti(i) - STAT.eps) & tg < (ti(i) + STAT.eps));
    if ( ~isempty(gdx) && gdx > 1)
        [R_N,R_E] = earth(lat(i));
        Tpr = diag([(R_N + h(i)), (R_E + h(i)) * cos(lat(i)), -1]);
        zp = Tpr * ([lat(i); lon(i); h(i);] - [GPS.lat(gdx); GPS.lon(gdx); GPS.h(gdx);]) ...
            + (DCMbn * STAT.larm);
        zv = (vel(i,:) - GPS.vel(gdx,:) - ((w_ie_n + w_en_n) .* (DCMbn * STAT.larm))' ...
            + (DCMbn * skewm(wb_corrected) * STAT.larm )' )';
        dtg = tg(gdx) - tg(gdx-1);
        UPDATE = [vel(i,:) lat(i) h(i) fn'];
        [KALMAN.F, KALMAN.G] = model(UPDATE, DCMbn, STAT.ab_corr, STAT.gb_corr);
            KALMAN.H = [ O I O O O ;
                O O Tpr O O ; ];
            KALMAN.R = diag([STAT.stdv STAT.stdm]).^2;
            KALMAN.z = [ zv' zp' ]';
        KALMAN.xp(1:9) = 0;         
        KALMAN = EKF_estimator(KALMAN, dtg,i,ti(i));
        
        %% INS/GNSS CORRECTIONS
        [q,DCMbn] = Quaternion_corrections(q_update,KALMAN);
        [roll,pitch,yaw] = Attitude_corrections(roll,pitch,yaw,KALMAN,i);
        [vel] = Velocity_corrections(vel,KALMAN,i);
        [lat,lon,h] = Position_corrections(lat,lon,h,KALMAN,i);
        [gb_dyn,ab_dyn] = Biases_corrections(gb_dyn,ab_dyn,KALMAN,i);
    end  
end

%% OUTPUTS
close(loop)
loop = waitbar(1,'DONE');
OUT.lon=lon;
OUT.lat=lat; 
OUT.h=h;
OUT.vel=vel; 
OUT.roll=roll; 
OUT.pitch=pitch; 
OUT.yaw=yaw;
ref.roll=roll;