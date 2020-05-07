% This function performs a simple extrapolation for attenuation coefficient
function y2=extrapolation(isotope)
    y1=isotope(2);
    y3=isotope(3);
    x1=isotope(4);
    x2=isotope(1);
    x3=isotope(5);
    k=(y3-y1)/(x3-x1);
    y2=(k*(x2-x1))+y1;
end