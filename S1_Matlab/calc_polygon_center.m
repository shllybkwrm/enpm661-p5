%%%
% Chris Wheatley
% This function claulates the gemotric center of a shape defined by line
%   segments
function [x1, y1] = calc_polygon_center(x,y)

%Input Arguments:
%   x: array of x coordinates
%   y: array of y coordinates
%Output Arguments:
%   x1: x location of centroid
%   y1: y location of centroid

      A = x(1:end-1).*y(2:end)-x(2:end).*y(1:end-1);
      As = sum(A)/2;
      x1 = (sum((x(2:end)+x(1:end-1)).*A)*1/6)/As;
      y1 = (sum((y(2:end)+y(1:end-1)).*A)*1/6)/As;
end