function varargout = AcuNavToolbox(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AcuNavToolbox_OpeningFcn, ...
                   'gui_OutputFcn',  @AcuNavToolbox_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function AcuNavToolbox_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);
function varargout = AcuNavToolbox_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


% --- Executes on button press in latitude.
function latitude_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in latitude','NumberTitle','off')
R2D=180/pi;
err_lat= ref.lat.*R2D - OUT.lat.*R2D;
save('SD_LAT.mat','err_lat')
save('SD_ti.mat','ti')
ERROR=sqrt(mean(err_lat.^2))
plot(ti , err_lat, 'b')
xlabel('Time [sec]')
ylabel('Error in latitude [degree]')
title ('Error in latitude')
grid on

% www = figure;
% R2D=180/pi;
% err_lat= ref.lat.*R2D - OUT.lat.*R2D;
% err_lon= ref.lon.*R2D - OUT.lon.*R2D;
% err_h= ref.h - OUT.h;
% ERROR=sqrt(mean(err_lat.^2));
% subplot(1,3,1)
% plot(ti , err_lat, 'b')
% xlabel({'Time [sec]';'(a) Latitude Error'})
% ylabel('Error in latitude [degree]')
% grid on
% subplot(1,3,2)
% plot(ti , err_lon, 'b')
% xlabel({'Time [sec]';'(b) Longitude Error'})
% ylabel('Error in longitude [degree]')
% grid on
% subplot(1,3,3)
% plot(ti , err_h, 'b')
% xlabel({'Time [sec]';'(c) Altitude Error'})
% ylabel('Error in altitude [meter]')
% 
% grid on
% set(www,'Units','Inches');
% set(www, 'Position',  [1, 1, 10, 4])
% pos = get(www,'Position');
% set(www,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(www,'STRAPDOWNfigure','-dpdf','-r0')


% --- Executes on button press in longitude.
function longitude_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in longitude','NumberTitle','off')
R2D=180/pi;
err_lon= ref.lon.*R2D - OUT.lon.*R2D;
save('SD_LON.mat','err_lon')
save('SD_ti_LON.mat','ti')
ERROR=sqrt(mean(err_lon.^2))
plot(ti , err_lon, 'b')
xlabel('Time [sec]')
ylabel('Error in longitude [degree]')
title ('Error in longitude')
grid on

% --- Executes on button press in altitude.
function altitude_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in altitude','NumberTitle','off')
R2D=180/pi;
err_h= ref.h - OUT.h;
save('SD_h.mat','err_h')
save('SD_ti_h.mat','ti')
ERROR=sqrt(mean(err_h.^2))
plot(ti , err_h, 'b')
xlabel('Time [sec]')
ylabel('Error in altitude [meter]')
title ('Error in altitude')
grid on

% --- Executes on button press in NorthVelocity.
function NorthVelocity_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in North Velocity','NumberTitle','off')
R2D=180/pi;
err_vel_N= ref.vel(:,1) - OUT.vel(:,1);
plot(ti , err_vel_N, 'b')
hold on
xlabel('Time [sec]')
ylabel('Error in North Velocity [m/s]')
title ('Error in North Velocity')
grid on

% --- Executes on button press in EastVelocity.
function EastVelocity_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in East Velocity','NumberTitle','off')
R2D=180/pi;
err_vel_E= ref.vel(:,2) - OUT.vel(:,2);
plot(ti , err_vel_E, 'b')
hold on
xlabel('Time [sec]')
ylabel('Error in East Velocity [m/s]')
title ('Error in East Velocity')
grid on

% --- Executes on button press in DownVelocity.
function DownVelocity_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in Down Velocity','NumberTitle','off')
R2D=180/pi;
err_vel_D= ref.vel(:,3) - OUT.vel(:,3);
plot(ti , err_vel_D, 'b')
hold on
xlabel('Time [sec]')
ylabel('Error in Down Velocity [m/s]')
title ('Error in Down Velocity')
grid on

% --- Executes on button press in Roll.
function Roll_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in Roll','NumberTitle','off')
R2D=180/pi;
err_Roll= ref.roll - OUT.roll;
plot(ti , err_Roll, 'b')
hold on
xlabel('Time [sec]')
ylabel('Error in Roll [rad]')
title ('Error in Roll')
grid on

% --- Executes on button press in Pitch.
function Pitch_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in Pitch','NumberTitle','off')
R2D=180/pi;
err_Pitch= ref.pitch - OUT.pitch;
plot(ti , err_Pitch, 'b')
hold on
xlabel('Time [sec]')
ylabel('Error in Pitch [rad]')
title ('Error in Pitch')
grid on

% --- Executes on button press in yaw.
function yaw_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Error in yaw','NumberTitle','off')
R2D=180/pi;
err_yaw = ref.yaw - OUT.yaw;
plot(ti , err_yaw, 'b')
hold on
xlabel('Time [sec]')
ylabel('Error in Yaw [rad]')
title ('Error in Yaw')
grid on

% --- Executes on button press in projection.
function projection_Callback(hObject, eventdata, handles)
OUT=handles.OUT;
ref=handles.ref;
ti=handles.ti;
figure('Name','Projection','NumberTitle','off')
R2D=180/pi;
xxx1=OUT.lat;
xxx2=OUT.lon;
save('SD_projection_lat.mat','xxx1')
save('SD_projection_lon.mat','xxx2')
plot(OUT.lat*R2D,OUT.lon*R2D, 'b')
hold on
xxx3=ref.lat;
xxx4=ref.lon;
save('ref_projection_lat.mat','xxx3')
save('ref_projection_lon.mat','xxx4')
plot(ref.lat*R2D ,ref.lon*R2D,'--k')
xlabel('lattitude [degree]')
ylabel('longitude [degree]')
grid on


function EKF_Callback(hObject, eventdata, handles)
VAR=get(get(handles.select,'SelectedObject'),'string');
if(VAR=='Trial 1')
    OPT=1;
end
if(VAR=='Trial 2')
    OPT=2;
end
if(VAR=='Trial 3')
    OPT=3;
end
[OUT, ref, ti] = Main_EKF(OPT);
handles.OUT=OUT;
guidata(hObject,handles)
handles.ref=ref;
guidata(hObject,handles)
handles.ti=ti;
guidata(hObject,handles)


function FAEKF_Callback(hObject, eventdata, handles)
VAR=get(get(handles.select,'SelectedObject'),'string');
if(VAR=='Data: M')
    OPT=1;
end
if(VAR=='Data: S')
    OPT=2;
end
if(VAR=='Data: T')
    OPT=3;
end
[OUT, ref, ti] = Main_FA(OPT);
handles.OUT=OUT;
guidata(hObject,handles)
handles.ref=ref;
guidata(hObject,handles)
handles.ti=ti;
guidata(hObject,handles)
