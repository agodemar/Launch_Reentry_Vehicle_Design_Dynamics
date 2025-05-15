function [h_fig] = plotBodyE(h_fig,shape,vXYZe,vEulerAngles,bodyAxesOptions,theView)
%
%   function plotBodyE(h_fig,shape,vXYZe,vEulerAngles,bodyAxesOptions,theView)
%   DESCRIPTION:        show a given aircraft shape in Earth-axes
%                       using CoG coordinates and Euler angles
%   INPUT:
%   h_fig               Matlab figure handle
%   shape               struct with fields shape.V, shape.F, shape.C
%                       (STL vertices and Face/Vertex connectivity infos)
%   vXYZe               triplet of CG coordinates (m)
%   vEulerAngles        Euler angles (rad)
%   bodyAxesOptions     body-axes settings
%                       bodyAxesOptions.show, =1 (true) if want to draw, body axes
%                       bodyAxesOptions.magX, magnifying factor of x-axis
%                       bodyAxesOptions.magY, magnifying factor of y-axis
%                       bodyAxesOptions.magZ, magnifying factor of z-axis
%                       bodyAxesOptions.lineWidth, arrow line width
%   theView             [azimuth elevation], the viewpoint in the scene
%
%   *******************************
%   Author: Agostino De Marco, Università di Napoli Federico II
%

% plot3(vXYZe(1),vXYZe(2),vXYZe(3));

%% Euler angles
psi   = vEulerAngles(1);
theta = vEulerAngles(2);
phi   = vEulerAngles(3);

%% DCM 
% Transf. matrix from Earth- to body-axes 
Tbe = angle2dcm(psi, theta, phi, 'ZYX')';

%% Vertices in body-axis coordinates
Vb = Tbe*shape.V';
Vb = Vb';

X0 = repmat(vXYZe,size(Vb,1),1);
Vb = Vb + X0;

%% plot body-axes
if (bodyAxesOptions.show)
    Xb = transpose( ...
        Tbe * (bodyAxesOptions.magX*[1;0;0]) ...
        );
    Yb = transpose( ...
        Tbe * (bodyAxesOptions.magY*[0;1;0]) ...
        );
    Zb = transpose( ...
        Tbe * (bodyAxesOptions.magZ*[0;0;1]) ...
        );
    quiver3( ...
        vXYZe(1),vXYZe(2),vXYZe(3), ...
        Xb(1),Xb(2),Xb(3), ...
        'AutoScale', 'off', 'Color',[1 0 0],'LineWidth',bodyAxesOptions.lineWidth ...
        );
    hold on
    quiver3( ...
        vXYZe(1),vXYZe(2),vXYZe(3), ...
        Yb(1),Yb(2),Yb(3), ...
        'AutoScale', 'off', 'Color',[0 1 0],'LineWidth',bodyAxesOptions.lineWidth ...
        ); hold on
    quiver3( ...
        vXYZe(1),vXYZe(2),vXYZe(3), ...
        Zb(1),Zb(2),Zb(3), ...
        'AutoScale', 'off', 'Color',[0 0 1],'LineWidth',bodyAxesOptions.lineWidth ...
        );
end
hold on

%% Display aircraft shape
p = patch('faces', shape.F, 'vertices' ,Vb);
set(p, 'facec', [1 0 0]);          
set(p, 'EdgeColor','none');
view(theView);
axis equal;
lighting phong

end

