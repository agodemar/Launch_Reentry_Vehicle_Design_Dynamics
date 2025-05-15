clear all; close all; clc

%% Load aircraft 3D model
shapeScaleFactor = 1.0;
% [V, F, C] = loadAircraftMAT('aircraft.mat', scale_factor);

% [V, F, C] = loadAircraftSTL('aircraft_pa24-250.stl', scale_factor);
% save shape in .mat format for future use
% shape.V = V; shape.F = F; shape.C = C;
% save('aircraft_pa24-250.mat', 'shape');

shape = loadAircraftMAT('aircraft_pa24-250.mat', shapeScaleFactor);

%% Setup the figure/scene
h_fig1 = figure(1);
grid on;
hold on;
light('Position',[1 0 -2],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

%% Display aircraft shape
p = patch('faces', shape.F, 'vertices' ,shape.V);
set(p, 'facec', [1 0 0]);          
set(p, 'EdgeColor','none');
theView = [-125 30];
view(theView);
axis equal;
lighting phong
hold on

%% Plot axes
xMax = 1.8*max(abs(shape.V(:,1)));
yMax = 1.8*max(abs(shape.V(:,2)));
zMax = 0.5*xMax;
quiver3( ...
    0,0,0, ...
    xMax,0,0, ...
    'r','linewidth',2.5 ...
); hold on;
quiver3( ...
    0,0,0, ...
    0,yMax,0, ...
    'g','linewidth',2.5 ...
); hold on;
quiver3( ...
    0,0,0, ...
    0,0,zMax, ...
    'b','linewidth',2.5 ...
); hold on;
