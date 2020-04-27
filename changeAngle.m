% This function ingests a node's location, angle, wheel radius, wheel distance, and wheel RPMs and generates a new
%   node location and angle/orientation.
function [newX,newY,newTheta,interpX,interpY,runningDist] = changeAngle(x,y,startAngle,rpmLeft,rpmRight,r,L)
% Converting wheel RPMs from revolutions per minute to meters per
%   second
    rpmLeft=rpmLeft*0.104719755;
    rpmRight=rpmRight*0.104719755;

    idx=0;
    interpX=zeros(1,10);
    interpY=zeros(1,10);
    runningDist=0;
    newTheta=wrapTo2Pi(startAngle);
    for dt=0:0.1:1
        idx=idx+1;
        dx=(r/2)*(rpmLeft+rpmRight)*cos(wrapTo2Pi(newTheta))*dt;
        dy=(r/2)*(rpmLeft+rpmRight)*sin(wrapTo2Pi(newTheta))*dt;
        newTheta=wrapTo2Pi(((r/L)*(rpmRight-rpmLeft))*dt)+newTheta;
        %newTheta=wrapTo2Pi((180/pi)*(((r/L)*(rpmRight-rpmLeft))*dt))+newTheta;
        interpX(idx)=dx+x;
        x=interpX(idx);
        interpY(idx)=dy+y;
        y=interpY(idx);
        runningDist=runningDist+sqrt((dx)^2+(dy)^2);
    end
    
    newX=interpX(end);
    newY=interpY(end);
    
end