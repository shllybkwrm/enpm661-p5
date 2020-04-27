% This function checks if a node exists within any of the obtacles, and
%   factors in the robot's clearance and radius
function [inObstacle] = obstacleCheckRigid(Obstacles,node,r,c)
    
    for i=1:1:length(Obstacles)
        h=Obstacles(i);
        In = inpolygon(node(1),node(2),h.XData+r+c,h.YData+r+c);
        In1= inpolygon(node(1),node(2),h.XData+r+c,h.YData-r-c);
        In2= inpolygon(node(1),node(2),h.XData-r-c,h.YData+r+c);
        In3= inpolygon(node(1),node(2),h.XData-r-c,h.YData-r-c);
        checker(i)=In;
        checker1(i)=In1;
        checker2(i)=In2;
        checker3(i)=In3;
    end
    if or(or(sum(checker)>=1,sum(checker1)>=1),or(sum(checker2)>=1,sum(checker3)>=1))==1
        inObstacle=1; 
    else
        if or(abs((5-abs(node(1))))<(r+c),abs((5-abs(node(2))))<(r+c))
            inObstacle=1; 
        else
            inObstacle=0;
        end
    end
    
    
    
end

