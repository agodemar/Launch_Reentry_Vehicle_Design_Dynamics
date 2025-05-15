clear all; close all; clc

%% Setup the figure/scene
h_fig1 = figure(1);

grid on;
hold on;
light('Position',[1 0 -2],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

%% Load aircraft shape
shapeScaleFactor = 1.0;
shape = loadAircraftMAT('aircraft_pa24-250.mat', shapeScaleFactor);

%% Set the aircraft in place
% Posision in Earth axes
vXYZe = [2,2,-3];
% psi, theta, phi -> 'ZYX'
vEulerAngles = convang([20,10,0],'deg','rad');
% Observer point-of-view
theView = [105 15];
% body axes settings
bodyAxesOptions.show = true;
bodyAxesOptions.magX = 2.0;
bodyAxesOptions.magY = 2.0;
bodyAxesOptions.magZ = 2.0;
bodyAxesOptions.lineWidth = 2.5;
plotBodyE(h_fig1, shape, vXYZe, vEulerAngles, bodyAxesOptions, theView);

%% Plot Earth axes
hold on;
xMax = max([abs(vXYZe(1)),5]);
yMax = max([abs(vXYZe(2)),5]);
zMax = 0.3*xMax; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig1, vXYZ0, vExtent);

%% draw CoG coordinate helper lines
hold on;
helperLinesOptions.lineColor = 'k';
helperLinesOptions.lineWidth = 1.5;
helperLinesOptions.lineStyle = ':';
plotPoint3DHelperLines(h_fig1, vXYZe, helperLinesOptions);

