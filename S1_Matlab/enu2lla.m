% WGS-84 east/north/up Cartesian coordinates to lat/lon/altitude coordinates
% Inputs:
    %  Input east/north/up data to be converted to lat/lon/alt - n x 3 matrix (n data points)
        % system reference lat/lon/alt: 1 x 3 vector: [lat lon alt]
        % enu data in meters
% Outputs
    % lat/lon/alt in n x 3 matrix (same size as input)
    
function lla = enu2lla(enu,sys_ref)
[p,q] = size(enu);
if q ~= 3
    enu = enu';
end
[p,q] = size(enu);
if q ~=3
    errordlg('Input Data Wrong Size in lla2enu')
    return
end
  npts = p;
  a = 6378137.0; % earth semimajor axis in meters
  f = 1/298.257223563; % reciprocal flattening
  b = a*(1-f);% semi-minor axis
 
  e = sqrt((a^2-b^2)/a^2);
  ePrime = sqrt((a^2-b^2)/b^2);
  e2 = 2*f -f^2; % eccentricity squared 

% get ECEF of reference
latRef = sys_ref(1)*pi/180;   % reference latitude converted to radians
lonRef = sys_ref(2)*pi/180;   % reference longitude converted to radians
altRef = sys_ref(3);          % reference altitude (meters)

chiRef = sqrt(1-e2*(sin(latRef)).^2); 
Xr = (a./chiRef +altRef).*cos(latRef).*cos(lonRef); 
Yr = (a./chiRef +altRef).*cos(latRef).*sin(lonRef); 
Zr = (a*(1-e2)./chiRef + altRef).*sin(latRef);

% get ECEF of data
e1 = enu(:,1);
n1 = enu(:,2);
u1 = enu(:,3);

X = -sin(lonRef)*e1 - cos(lonRef)*sin(latRef)*n1 + cos(lonRef)*cos(latRef)*u1 + Xr;
Y =  cos(lonRef)*e1 - sin(lonRef)*sin(latRef)*n1 + cos(latRef)*sin(lonRef)*u1 + Yr;  
Z = cos(latRef)*n1 + sin(latRef)*u1 + Zr;
aaa = 1;

% now convert ECEF to LLA using iterative scheme
  
  p = sqrt(X.^2 + Y.^2);
  lambda = atan2(Y,X);
  phiLoop(:,1) = atan2(Z,p.*(1-e^2));
  NLoop(:,1) = a./sqrt(1-(e*sin(phiLoop(:,1))).^2);
  hLoop(:,1) = zeros(npts,1);
  
  doneTest = 1000;
  k = 1;
  while doneTest(k) > 1e-10      % doneTest is difference between successive latitude estimates
      k = k+1;
      hLoop(:,k) = p./cos(phiLoop(:,k-1)) - NLoop(:,k-1);
      phiLoop(:,k) = atan2(Z,p.*(1-e^2*(NLoop(:,k-1)./(NLoop(:,k-1)+hLoop(:,k)))));
      NLoop(:,k) = a*ones(npts,1)./sqrt(1-(e*sin(phiLoop(:,k))).^2);
      doneTest(k) = max(abs(phiLoop(:,k) - phiLoop(:,k-1)));
  end
  
 lla(:,1) = phiLoop(:,end)*180/pi;
 lla(:,2) = lambda*180/pi;
 lla(:,3) = hLoop(:,end);

