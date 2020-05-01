function [a,b] = findLinePts(xc,yc)

%Input Arguments:
%   xc,yc: center of attenuating obstacle
%Output Arguments:
%   a: min point on perpendicular line bisecting obstacle
%   b: max point on perpendicular line bisecting obstacle
    global x_source y_source map_height;
    
    % perpendicular slope = -1/m
    m = (-1)./((y_source - yc) / (x_source - xc));
    yint = yc - m*xc;  % y-intercept: b=y-mx using cask center
    x0 = (0-yint)/m;  % x-intercept: x=(y-b)/m
    xmax_local = (map_height-yint)/m;
    %print(m, b, x0, xmax_local)
    a = [x0,0];
    b = [xmax_local,map_height];
end