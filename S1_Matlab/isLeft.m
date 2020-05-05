%%%
% Shelly Bagchi
% Function to determine whether each point is on the left or right of a
% line bisecting the obstacle in question.  The line is defined by points
% found from findLinePts(xc,yc).

function [left] = isLeft(a,b)

%Input Arguments:
%   a: lower point on line
%   b: higher point on line
%   c: point to check
%Output Arguments:
%   left: boolean matrix for if each point is left of the a-b line or not
    global X Y
    
    left = ((b(1) - a(1))*(Y - a(2)) - (b(2) - a(2))*(X - a(1))) > 0;
end