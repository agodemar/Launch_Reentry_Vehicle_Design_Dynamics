clear all; close all; clc

%% Prepare to solve kinematics equations

t_fin = 100.0; % Simulation final time

% Initial conditions
psi0 = 0; theta0 = 0; phi0 = 0; % Euler angles
Z0 = -1000;

%% Anonymous functions

% Tonneau, some reference values used to build
% three appropriate time histories of p, q, r
p_max = convangvel(2.0,'deg/s','rad/s'); % rad/s
q_max = convangvel(1.0,'deg/s','rad/s'); % rad/s 
r_max = convangvel(1.2,'deg/s','rad/s'); % rad/s

vBreakPointsP(1,:) = [0, 0.030*t_fin, 0.08*t_fin, 0.20*t_fin, 0.25*t_fin, 0.35*t_fin, 0.60*t_fin,  0.67*t_fin, 0.75*t_fin,  t_fin];
vBreakPointsP(2,:) = [0, 0.025*p_max, 0.70*p_max, 1.00*p_max, 1.00*p_max, 0,         -1.00*p_max, -1.00*p_max, 0,           0];
p = @(t) ...
    interp1( vBreakPointsP(1,:), vBreakPointsP(2,:), t, 'pchip' );

vBreakPointsQ(1,:) = [0, 0.030*t_fin, 0.10*t_fin, 0.20*t_fin, 0.48*t_fin,  0.60*t_fin, 0.70*t_fin, t_fin];
vBreakPointsQ(2,:) = [0, 0.025*q_max, 0.75*q_max, 1.00*q_max, 1.00*q_max, -0.40*q_max, 0,          0];
q = @(t) ...
    interp1( vBreakPointsQ(1,:), vBreakPointsQ(2,:), t, 'pchip' );

vBreakPointsR(1,:) = [0, 0.030*t_fin, 0.10*t_fin, 0.20*t_fin, 0.50*t_fin, 0.60*t_fin, t_fin];
vBreakPointsR(2,:) = [0, 0.025*r_max, 0.75*r_max, 1.00*r_max, 1.00*r_max, 0,          0];
r = @(t) ...
    interp1( vBreakPointsR(1,:), vBreakPointsR(2,:), t, 'pchip' );

% Looping
% p = @(t) 0.; q = @(t) 1.; r = @(t) 0.;

% Tonneau, some reference values used to build
% three appropriate time histories of u, v, w
u0 = convvel(380.0,'km/h','m/s'); % m/s
v0 = convvel(  0.0,'km/h','m/s'); % m/s 
w0 = convvel(  0.0,'km/h','m/s'); % m/s 

vBreakPointsU(1,:) = [0, t_fin/30, t_fin/10, t_fin/5, 0.8*t_fin, t_fin];
vBreakPointsU(2,:) = [u0,  0.8*u0,   0.7*u0,      u0,    1.1*u0,    u0];
u = @(t) ...
    interp1( vBreakPointsU(1,:), vBreakPointsU(2,:), t, 'pchip' );
% assume alpha = beta = 0
v = @(t) ...
    0;
w = @(t) ...
    0;

%% Anonymous functions
% RHS of Euler angles evolution equations, see eq. (2.27)
dPhiThetaPsidt = @(t, x) ... % x -> (phi, theta, psi)
    [1, sin(x(1)).*sin(x(2))./cos(x(2)), ...
            cos(x(1)).*sin(x(2))./cos(x(2)); ...
     0, cos(x(1)), -sin(x(1)); ...
     0, sin(x(1))./cos(x(2)), cos(x(1))./cos(x(2))   ...
         ] * [p(t); q(t); r(t)];

%% Solution of Gimbal equations
options = odeset( ...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9*ones(1,3) ...
    );
vPhiThtetaPsi0 = [phi0, theta0, psi0];
[vTime, vPhiThetaPsi] = ode45(dPhiThetaPsidt, [0 t_fin], vPhiThtetaPsi0, options);
N = length(vPhiThetaPsi);

%% velocity components
for i=1:numel(vTime)
    vU(i) = u(vTime(i));
    vV(i) = v(vTime(i));
    vW(i) = w(vTime(i));
end

%% Plots

% Euler angles time histories
figure(1)
subplot 121,
plot( ...
    vTime,convang(vPhiThetaPsi(:,1),'rad','deg'),'-', ...
    vTime,convang(vPhiThetaPsi(:,2),'rad','deg'),'--', ...
    vTime,convang(vPhiThetaPsi(:,3),'rad','deg'),'-' ...
    )
legend('\phi','\theta','\psi')
xlabel('t (s)'); ylabel('(deg)')
title('Euler angles')
% set(gca,'fontname','cambria','fontsize',15)

% Angular velocity components
subplot 122
plot( ...
    vTime, convangvel(p(vTime),'rad/s','deg/s'), ...
    vTime, convangvel(q(vTime),'rad/s','deg/s'), ...
    vTime, convangvel(r(vTime),'rad/s','deg/s'), ...
    vBreakPointsP(1,:), convangvel(vBreakPointsP(2,:),'rad/s','deg/s'), '.', ...
    vBreakPointsQ(1,:), convangvel(vBreakPointsQ(2,:),'rad/s','deg/s'), '.', ...
    vBreakPointsR(1,:), convangvel(vBreakPointsR(2,:),'rad/s','deg/s'), '.' ...
    )
% axis([0 20 -.3 .6])
legend('p(t)','q(t)','r(t)')
xlabel('t (s)'); ylabel('(deg/s)')
title('Angular velocity components in body axes')
% set(gca,'fontname','cambria','fontsize',15)

%% Integrate Navigation equations
% Time interpolation function for known Euler angles histories
fPhi = @(t) ...
    interp1(vTime,vPhiThetaPsi(:,1),t);
fTheta = @(t) ...
    interp1(vTime,vPhiThetaPsi(:,2),t);
fPsi = @(t) ...
    interp1(vTime,vPhiThetaPsi(:,3),t);
% RHS of navigation equations
dPosEdt = @(t,Pos) ...
    transpose(angle2dcm(fPsi(t),fTheta(t),fPhi(t),'ZYX'))*[u(t);v(t);w(t)]; %  + 0.*Pos;

%% Solution of navigation equations
options = odeset( ...
    'RelTol', 1e-3, ...
    'AbsTol', 1e-3*ones(3,1) ...
    );
PosE0 = [0;0;0];
[vTime2, vPosE] = ode45(dPosEdt, vTime, PosE0, options);
N = length(vPosE);
           
vXe = vPosE(:,1); vYe = vPosE(:,2); vZe = Z0 + vPosE(:,3);

% CG coordinates time history
figure(2)
plot( ...
    vTime2,vXe, '-',...
    vTime2,vYe, '-.', ...
    vTime2,vZe, '--' ...
    )
% hold on, view(3)
legend('x_{G,E}(t)','y_{G,E}(t)','z_{G,E}(t)')
xlabel('t (s)'); ylabel('(m)');
title('CG coordinates in Earth axes');

%% Setup the figure/scene for 3D visualization
h_fig3 = figure(3);
grid on;
hold on;
light('Position',[1 0 -4],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
daspect([1 1 1]);

%% Load aircraft shape
shapeScaleFactor = 350.0;
%shape = loadAircraftMAT('aircraft_pa24-250.mat', scale_factor);
shape = loadAircraftMAT('aircraft_mig29.mat', shapeScaleFactor);

mXYZe = [vPosE(:,1),vPosE(:,2),vPosE(:,3)+Z0];
mEulerAngles = [vPhiThetaPsi(:,3),vPhiThetaPsi(:,2),vPhiThetaPsi(:,1)];

%% Settings
% General settings
options.samples = [1,61,111,141:50:numel(vTime)]; %[1,40,80,120,160,200,250,300,320,numel(vTime)];
options.theView = [105 15];

% body axes settings
options.bodyAxes.show = true;
options.bodyAxes.magX = 1.5*shapeScaleFactor;
options.bodyAxes.magY = 2.0*shapeScaleFactor;
options.bodyAxes.magZ = 2.0*shapeScaleFactor;
options.bodyAxes.lineWidth = 2.5;

% helper lines
options.helperLines.show = true;
options.helperLines.lineStyle = ':';
options.helperLines.lineColor = 'k';
options.helperLines.lineWidth = 1.5;

% trajectory
options.trajectory.show = true;
options.trajectory.lineStyle = '-';
options.trajectory.lineColor = 'k';
options.trajectory.lineWidth = 1.5;

%% Plot body and trajectory
plotTrajectoryAndBodyE(h_fig3, shape, mXYZe, mEulerAngles, options);

%% Plot Earth axes
hold on;
xMax = max([max(abs(mXYZe(:,1))),5]);
yMax = max([max(abs(mXYZe(:,2))),5]);
zMax = 0.05*xMax; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig3, vXYZ0, vExtent);
xlabel('x_E (m)'); ylabel('y_E (m)'); zlabel('z_E (m)')
hold off

%% Save variables for post processing

data_pqr = [vTime,p(vTime)*57.3,q(vTime)*57.3,r(vTime)*57.3];
filename_pqr = 't_pqr.txt';
fileID_pqr = fopen(filename_pqr,'w');
%fprintf(fileID_pqr,'%s\n','time (s), p (deg/s), q (deg/s), r (deg/s)');
fprintf(fileID_pqr,'%s\n','time p q r');
fclose(fileID_pqr);
save(filename_pqr,'data_pqr','-ASCII','-APPEND')

data_uvw = [vTime,u(vTime)];
filename_uvw = 't_uvw.txt';
fileID_uvw = fopen(filename_uvw,'w');
%fprintf(fileID_uvw,'%s\n','time (s), u (m/s)');
fprintf(fileID_uvw,'%s\n','time u');
fclose(fileID_uvw);
save(filename_uvw,'data_uvw','-ASCII','-APPEND')

% data_quat = [vTime,vQuat];
% filename_quat = 't_quat.txt';
% fileID_quat = fopen(filename_quat,'w');
% fprintf(fileID_quat,'%s\n','time q0 qx qy qz');
% fclose(fileID_quat);
% save(filename_quat,'data_quat','-ASCII','-APPEND')

data_euler = [vTime,vPhiThetaPsi(:,3)*57.3,vPhiThetaPsi(:,2)*57.3,vPhiThetaPsi(:,1)*57.3];
filename_euler = 't_euler.txt';
fileID_euler = fopen(filename_euler,'w');
%fprintf(fileID_euler,'%s\n','time (s), psi (deg), theta (deg), phi (deg)');
fprintf(fileID_euler,'%s\n','time psi theta phi');
fclose(fileID_euler);
save(filename_euler,'data_euler','-ASCII','-APPEND')

data_xyz = [vTime,vXe,vYe,vZe];
filename_xyz = 't_xyz.txt';
fileID_xyz = fopen(filename_xyz,'w');
%fprintf(fileID_xyz,'%s\n','time (s), Xe (m), Ye (m), Ze (m)');
fprintf(fileID_xyz,'%s\n','time Xe Ye Ze');
fclose(fileID_xyz);
save(filename_xyz,'data_xyz','-ASCII','-APPEND')
