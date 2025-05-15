function [h_fig] = plotBodyInEarthAxesE(h_fig,vXYZe,vEulerAngles,scale_factor,theView)
%
%   function plotBodyInEarthAxes(h_fig,vXe,vYe,vZe,vEulerAngles,scale_factor,theView)
%   INPUT:
%   vXYZe               triplet of CG coordinates (m)
%   vEulerAngles        Euler angles (rad)
%   scale_factor        normalization factor (scalar)
%                       (related to body aircraft dimension, >1 magnifies the body)
%   theView             [azimuth elevation], the viewpoint in the scene
%
%   *******************************
%   Author: Agostino De Marco, Università di Napoli Federico II
%

%% Read aircraft data file 'aircraft.mat' and prepare vertices
[V, F, C] = loadAircraft('aircraft.mat', scale_factor);

% tweak here 3D model position
% V(:,1) = V(:,1)-0.5;

%% Figure presets
clf(h_fig);
h_fig = plot3(vXYZe(1),vXYZe(2),vXYZe(3));
grid on;
hold on;
light('Position',[1 0 -5],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

%% Euler angles (sign tweak)
psi   = vEulerAngles(1);
theta = vEulerAngles(2);
phi   = vEulerAngles(3);

%% DCM 
% Transf. matrix from Earth- to body-axes 
Tbe = angle2dcm(psi, theta, phi, 'ZYX')';

%% Vertices in body-axis coordinates
Vb = Tbe*V';
Vb = Vb';

X0 = repmat(vXYZe,size(Vb,1),1);
Vb = Vb + X0;

%% plot body-axes
Xb = transpose( ...
    Tbe * ((2.*scale_factor)*[1;0;0]) ...
    );
Yb = transpose( ...
    Tbe * ((2.*scale_factor)*[0;1;0]) ...
    );
Zb = transpose( ...
    Tbe * ((2.*scale_factor)*[0;0;1]) ...
    );
quiver3( ...
    vXYZe(1),vXYZe(2),vXYZe(3), ...
    Xb(1),Xb(2),Xb(3), ...
    'r','linewidth',2.5 ...
); 
hold on
quiver3( ...
    vXYZe(1),vXYZe(2),vXYZe(3), ...
    Yb(1),Yb(2),Yb(3), ...
    'g','linewidth',2.5 ...
); hold on
quiver3( ...
    vXYZe(1),vXYZe(2),vXYZe(3), ...
    Zb(1),Zb(2),Zb(3), ...
    'b','linewidth',2.5 ...
); hold on

%% Display aircraft shape
p = patch('faces', F, 'vertices' ,Vb);
set(p, 'facec', [1 0 0]);          
set(p, 'EdgeColor','none');
view(theView);
axis equal;
lighting phong

%% Plot Earth axes
xMax = max([abs(vXYZe(1)),5]);
yMax = max([abs(vXYZe(2)),5]);
zMax = xMax; % max([abs(max(vXYZe(1))),0.18*xMax]);
quiver3( ...
    0,0,0, ...
    abs(1.1*xMax),0,0, ...
    'color',[1 0.3 0.3],'linewidth',2.5 ...
); hold on;
quiver3( ...
    0,0,0, ...
    0,abs(1.1*yMax),0, ...
    'color',[0.3 1 0.3],'linewidth',2.5 ...
); hold on;
quiver3( ...
    0,0,0, ...
    0,0,zMax, ...
    'color',[0.3 0.3 1],'linewidth',2.5 ...
); hold on;

end

