function [ shape ] = loadAircraftMAT( fileName, scale_factor )
%
%   function loadAircraftMAT( fileName, scale_factor )
%   INPUT:
%   fileName            (string) MAT-file name containing original matrices
%                       V (vertices), F (faces), C (connectivity)
%                       see: http://www.mathworks.com/matlabcentral/fileexchange/3642
%                       for functions that translate STL files into .mat with Vertices,
%                       Faces and Connectivity infos
%   scale_factor        (double, scalar) scale factor
%                       (> 1 ==> magnifies the body)
%
%   *******************************
%   Author: Agostino De Marco, Università di Napoli Federico II
%

load(fileName,'-mat', 'shape');
% shape.V; % vertices
% shape.C; % cells
% shape.F; % faces

% sort-of center the shape
shape.V(:,1) = shape.V(:,1)-round(sum(shape.V(:,1))/size(shape.V,1));
shape.V(:,2) = shape.V(:,2)-round(sum(shape.V(:,2))/size(shape.V,1));
shape.V(:,3) = shape.V(:,3)-round(sum(shape.V(:,3))/size(shape.V,1));

% scale the shape
Xb_nose_tip = max(abs(shape.V(:,1)));
shape.V = (shape.V).*(scale_factor./Xb_nose_tip);

end

