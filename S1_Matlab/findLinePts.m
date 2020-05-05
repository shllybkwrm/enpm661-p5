%%%
% Shelly Bagchi
% Function to find min and max points on line bisecting obstacle, 
% perpendicular to radiation source.
% Line points can be used to determine which side of the line a point is
% on, using the function isLeft(a,b)

function [a,b] = findLinePts(xc,yc, xs,ys)

%Input Arguments:
%   xc,yc: center of attenuating obstacle
%   xs,ys: center of source.  Defaults to main source if no vars passed or []
%Output Arguments:
%   a: min point on perpendicular line bisecting obstacle
%   b: max point on perpendicular line bisecting obstacle
    global x_source y_source map_height;
    if ~exist('xs','var')
        xs = x_source;
    end
    if ~exist('ys','var')
        ys = y_source;
    end
    
    % perpendicular slope = -1/m
    m = (-1)./((ys - yc) / (xs - xc));
    yint = yc - m*xc;  % y-intercept: b=y-mx using cask center
    x0 = (0-yint)/m;  % x-intercept: x=(y-b)/m
    xmax_local = (map_height-yint)/m;
    %print(m, b, x0, xmax_local)
    a = [x0,0];
    b = [xmax_local,map_height];
end