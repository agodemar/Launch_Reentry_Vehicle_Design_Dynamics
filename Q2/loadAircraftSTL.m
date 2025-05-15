function [ V, F, C ] = loadAircraftSTL( fileName, scale_factor )
%
%   function loadAircraft( fileName, scale_factor )
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

[F, V, C] = read_stl(fileName);

% sort-of center the shape
V(:,1) = V(:,1)-round(sum(V(:,1))/size(V,1));
V(:,2) = V(:,2)-round(sum(V(:,2))/size(V,1));
V(:,3) = V(:,3)-round(sum(V(:,3))/size(V,1));

% scale the shape
Xb_nose_tip = max(abs(V(:,1)));
V = V.*(scale_factor./Xb_nose_tip);

end

