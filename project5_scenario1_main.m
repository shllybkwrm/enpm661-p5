
% Chris Wheatley 
% ENPM661 Spring 2020
% Project #5 

close all; 
clear all;

% % Solicit input node configuration from user
% fprintf('\n');
% prompt = 'Enter x,y START node location in meters and starting orientation in deg. (e.g. [1,1,30]): ';
% start_node = input(prompt);
% fprintf('\n');
% 
% % Solicit goal node configuration from user
% fprintf('\n');
% prompt1 = 'Enter x,y GOAL node location in meters (e.g. [10,9]): ';
% goal_node = input(prompt1);
% fprintf('\n');
% 
% % Calulated Paramaters From Datasheet
% r=(354/2)/1000;
% L=2*r;
% wheelLength=pi*(76/1000);
% wheelRad=(76/2)/1000;
% 
% % Solicit obstacle clearance from user
% fprintf('\n');
% prompt3 = 'Enter obstacle clearance (meters): ';
% c = input(prompt3);
% fprintf('\n');
% 
% % Solicit wheel RPMs from user
% fprintf('\n');
% prompt3 = 'Enter wheel RPMs [rpm1, rpm2]: ';
% rpms = input(prompt3);
% fprintf('\n');

xmin=0; ymin=0;
xmax=450; ymax=232;
fig=figure; hold on; axis equal;
% Define black border around action movement area
h_border=line([xmin xmax xmax xmin xmin],[ymin ymin ymax ymax ymin],'Color','black','LineWidth',2);
xlabel('X Coordinates (meters)'); ylabel('Y Coordinates (meters)');
title('Scenario 1 RRT (Differential Drive, Rigid Robot) (Pink Circle = Start; Pink Triangle = Goal; Black Circle = "Close Enough to Goal" Region)','FontSize',14);

xl = [xmin xmax]; 
yl = [ymin ymax];

air_density = 1205; % Dry air near sea level in g/m^3
concrete_density = 3150000; % Density of concrete in g/m^3
steel_density = 7190000; % Density of steel in g/m^3
x_source=430;
y_source=222;


%%%%%%%%%%%%%%%%%%%%%%% START of LU's EDITS %%%%%%%%%%%%%%%%%%%%%%%%5
map_height=232; %m
map_width=450; %m
[X,Y]=meshgrid([1:1:map_width],[1:1:map_height]);
X=flip(X);
Y=flip(Y);

% Find x,y centroid of each obstacle
[x1,y1]=calc_polygon_center([15 15 36 36 56 56 15],[108 168 168 130 130 108 108]);
[x2,y2]=calc_polygon_center([40 40 96 96 40],[192 216 216 192 192]);
[x3,y3]=calc_polygon_center([89 89 112 112 89],[87 112 112 87 87]);
[x4,y4]=calc_polygon_center([77 77 99 99 122 122 145 145 77],[140 183 183 168 168 183 183 140 140]);
[x5,y5]=calc_polygon_center([93 93 190 190 93],[5 53 53 5 5]);
[x6,y6]=calc_polygon_center([154 154 225 225 154],[144 200 200 144 144]);
[x7,y7]=calc_polygon_center([350 350 426 426 350],[60 101 101 60 60]);
[x8,y8]=calc_polygon_center([219 219 200 200 219 219 238 238 219],[36 46 46 67 67 77 77 36 36]);
[x9,y9]=calc_polygon_center([267 267 293 293 267 267 338 338 267],[139 160 160 175 175 206 206 139 139]);
[x10,y10]=calc_polygon_center([358 358 439 439 358],[127 204 204 127 127]);
[x11,y11]=calc_polygon_center([53 39 55 77 66 53],[4 23 37 14 4 4]);
x11=round(x11); y11=round(y11);
[x12,y12]=calc_polygon_center([251 251 312 312 251],[51 113 91 41 51]);

h1=line([15 15 36 36 56 56 15],[108 168 168 130 130 108 108],'Color','white','LineWidth',3);
h2=line([40 40 96 96 40],[192 216 216 192 192],'Color','white','LineWidth',3);
h3=line([89 89 112 112 89],[87 112 112 87 87],'Color','white','LineWidth',3);
h4=line([77 77 99 99 122 122 145 145 77],[140 183 183 168 168 183 183 140 140],'Color','white','LineWidth',3);
h5=line([93 93 190 190 93],[5 53 53 5 5],'Color','white','LineWidth',3);
h6=line([154 154 225 225 154],[144 200 200 144 144],'Color','white','LineWidth',3);
h7=line([350 350 426 426 350],[60 101 101 60 60],'Color','white','LineWidth',3);
h8=line([219 219 200 200 219 219 238 238 219],[36 46 46 67 67 77 77 36 36],'Color','white','LineWidth',3);
h9=line([267 267 293 293 267 267 338 338 267],[139 160 160 175 175 206 206 139 139],'Color','white','LineWidth',3);
h10=line([358 358 439 439 358],[127 204 204 127 127],'Color','white','LineWidth',3);
h11=line([53 39 55 77 66 53],[4 23 37 14 4 4],'Color','white','LineWidth',3);
h12=line([251 251 312 312 251],[51 113 91 41 51],'Color','white','LineWidth',3,'LineStyle','--');

cesium_137=[0.662,8.04e-6,7.065e-6,0.6,0.8];
concrete = [0.662,8.062e-6,6.083e-6, 0.6,0.8];
steel=[0.323,2.875e-6,2.953e-6,0.3,0.4];   % <-----NEED TO CHANGE THIS
interaction_coefficient=extrapolation(cesium_137);
interaction_coefficient_concrete=extrapolation(concrete);
interaction_coefficient_steel=extrapolation(steel);
total_miu= interaction_coefficient*air_density; % 1/m
total_miu_concrete= interaction_coefficient_concrete*concrete_density; % 1/m
total_miu_steel=interaction_coefficient_steel*steel_density;

for rows=1:1:map_height
    for cols=1:1:map_width
        R(rows,cols)=sqrt(((Y(rows,cols)-y_source)^2)+((X(rows,cols)-x_source)^2));
        R1(rows,cols)=sqrt(((Y(rows,cols)-y1)^2)+((X(rows,cols)-x1)^2));
        R2(rows,cols)=sqrt(((Y(rows,cols)-y2)^2)+((X(rows,cols)-x2)^2));
        R3(rows,cols)=sqrt(((Y(rows,cols)-y3)^2)+((X(rows,cols)-x3)^2));
        R4(rows,cols)=sqrt(((Y(rows,cols)-y4)^2)+((X(rows,cols)-x4)^2));
        R5(rows,cols)=sqrt(((Y(rows,cols)-y5)^2)+((X(rows,cols)-x5)^2));
        R6(rows,cols)=sqrt(((Y(rows,cols)-y6)^2)+((X(rows,cols)-x6)^2));
        R7(rows,cols)=sqrt(((Y(rows,cols)-y7)^2)+((X(rows,cols)-x7)^2));
        R8(rows,cols)=sqrt(((Y(rows,cols)-y8)^2)+((X(rows,cols)-x8)^2));
        R9(rows,cols)=sqrt(((Y(rows,cols)-y9)^2)+((X(rows,cols)-x9)^2));
        R10(rows,cols)=sqrt(((Y(rows,cols)-y10)^2)+((X(rows,cols)-x10)^2));
        
        R11(rows,cols)=sqrt(((Y(rows,cols)-y11)^2)+((X(rows,cols)-x11)^2));
        R12(rows,cols)=sqrt(((Y(rows,cols)-y12)^2)+((X(rows,cols)-x12)^2));
    end
end

dist1=sqrt(((h1.XData-x1).^2)+((h1.YData-y1).^2));
dist2=sqrt(((h2.XData-x2).^2)+((h2.YData-y2).^2));
dist3=sqrt(((h3.XData-x3).^2)+((h3.YData-y3).^2));
dist4=sqrt(((h4.XData-x4).^2)+((h4.YData-y4).^2));
dist5=sqrt(((h5.XData-x5).^2)+((h5.YData-y5).^2));
dist6=sqrt(((h6.XData-x6).^2)+((h6.YData-y6).^2));
dist7=sqrt(((h7.XData-x7).^2)+((h7.YData-y7).^2));
dist8=sqrt(((h8.XData-x8).^2)+((h8.YData-y8).^2));
dist9=sqrt(((h9.XData-x9).^2)+((h9.YData-y9).^2));
dist10=sqrt(((h10.XData-x10).^2)+((h10.YData-y10).^2));
dist11=sqrt(((h11.XData-x11).^2)+((h11.YData-y11).^2));
dist12=sqrt(((h12.XData-x12).^2)+((h12.YData-y12).^2));
R1(R1>max(dist1))=0;
R2(R2>max(dist2))=0;
R3(R3>max(dist3))=0;
R4(R4>max(dist4))=0;
R5(R5>max(dist5))=0;
R6(R6>max(dist6))=0;
R7(R7>max(dist7))=0;
R8(R8>max(dist8))=0;
R9(R9>max(dist9))=0;
R10(R10>max(dist10))=0;
R11(R11>max(dist11))=0;
% R12(R12<max(dist12))=0;

source_strength=(9.45601235e17)./(4.*pi.*R.^2);
%source_strength_steel_1 = (9.45601235e17)./(4.*pi.*R11.^2);
source_strength_steel_2 = (9.45601235e17)./(4.*pi.*R12.^2);
% source_strength1=(9.45601235e17)./(4.*pi.*R1.^2);
% source_strength2=(9.45601235e17)./(4.*pi.*R2.^2);
% source_strength3=(9.45601235e17)./(4.*pi.*R3.^2);
% source_strength4=(9.45601235e17)./(4.*pi.*R4.^2);
% source_strength5=(9.45601235e17)./(4.*pi.*R5.^2);
% source_strength6=(9.45601235e17)./(4.*pi.*R6.^2);
% source_strength7=(9.45601235e17)./(4.*pi.*R7.^2);
% source_strength8=(9.45601235e17)./(4.*pi.*R8.^2);
% source_strength9=(9.45601235e17)./(4.*pi.*R9.^2);
% source_strength10=(9.45601235e17)./(4.*pi.*R10.^2);

attenuation_at_R=exp(-total_miu.*abs(R));
attenuation_at_R1=exp(-total_miu_concrete.*abs(R1));
attenuation_at_R2=exp(-total_miu_concrete.*abs(R2));
attenuation_at_R3=exp(-total_miu_concrete.*abs(R3));
attenuation_at_R4=exp(-total_miu_concrete.*abs(R4));
attenuation_at_R5=exp(-total_miu_concrete.*abs(R5));
attenuation_at_R6=exp(-total_miu_concrete.*abs(R6));
attenuation_at_R7=exp(-total_miu_concrete.*abs(R7));
attenuation_at_R8=exp(-total_miu_concrete.*abs(R8));
attenuation_at_R9=exp(-total_miu_concrete.*abs(R9));
attenuation_at_R10=exp(-total_miu_concrete.*abs(R10));
attenuation_at_R11=exp(-total_miu_concrete.*abs(R11));
attenuation_at_R12=exp(-.001.*abs(R12));

total_concrete_attenuation=attenuation_at_R1+attenuation_at_R2+attenuation_at_R3+attenuation_at_R4+attenuation_at_R5+attenuation_at_R6+attenuation_at_R7+attenuation_at_R8+attenuation_at_R9+attenuation_at_R10+attenuation_at_R11;

response_function=1.835e-3*cesium_137(1)*interaction_coefficient;
response_function_steel_1=1.835e-3*steel(1)*interaction_coefficient_steel;
Exposure1=response_function.*source_strength.*attenuation_at_R;

Exposure2=Exposure1.*attenuation_at_R1.*attenuation_at_R2.*attenuation_at_R3.*attenuation_at_R4.*attenuation_at_R5.*attenuation_at_R6.*attenuation_at_R7.*attenuation_at_R8.*attenuation_at_R9.*attenuation_at_R10+attenuation_at_R11;
%Exposure=Exposure1.*total_concrete_attenuation;
%Exposure_steel_1=response_function_steel_1.*source_strength_steel_1.*attenuation_at_R11;
Exposure_steel_2=response_function_steel_1.*source_strength_steel_2.*attenuation_at_R12;
%Exposure=Exposure2+Exposure_steel_2;
Exposure=(Exposure1+Exposure_steel_2).*attenuation_at_R1.*attenuation_at_R2.*attenuation_at_R3.*attenuation_at_R4.*attenuation_at_R5.*attenuation_at_R6.*attenuation_at_R7.*attenuation_at_R8.*attenuation_at_R9.*attenuation_at_R10+attenuation_at_R11;

%%%%%%%%%%%%%%%%%%%%% END of LU's EDITS %%%%%%%%%%%%%%%%%%%%


Exposure_tmp=Exposure;
Exposure_tmp(and(Exposure<1e9,Exposure>=5.5e8))=16;
Exposure_tmp(and(Exposure<5.5e8,Exposure>=1e8))=15;
Exposure_tmp(and(Exposure<1e8,Exposure>=5.5e7))=14;
Exposure_tmp(and(Exposure<5.5e7,Exposure>=1e7))=13;
Exposure_tmp(and(Exposure<1e7,Exposure>=5.5e6))=12;
Exposure_tmp(and(Exposure<5.5e6,Exposure>=1e6))=11;
Exposure_tmp(and(Exposure<1e6,Exposure>=5.5e5))=10;
Exposure_tmp(and(Exposure<5.5e5,Exposure>=1e5))=9;
Exposure_tmp(and(Exposure<1e5,Exposure>=5.5e4))=8;
Exposure_tmp(and(Exposure<5.5e4,Exposure>=1e4))=7;
Exposure_tmp(and(Exposure<1e4,Exposure>=5.5e3))=6;
Exposure_tmp(and(Exposure<5.5e3,Exposure>=1e3))=5;
Exposure_tmp(and(Exposure<1e3,Exposure>=5.5e2))=4;
Exposure_tmp(and(Exposure<5.5e2,Exposure>=1e2))=3;
Exposure_tmp(and(Exposure<1e2,Exposure>=5.5e1))=2;
Exposure_tmp(Exposure<5.5e1)=1;

hold on;
hh=surf(X,Y,Exposure_tmp,'FaceAlpha',.5,'EdgeAlpha',.5);
view(2)
xlim([xmin xmax]); 
ylim([ymin ymax]);
hold on
z = get(hh,'ZData');
set(hh,'ZData',z-15)

% Define obstacle space
h1=line([15 15 36 36 56 56 15],[108 168 168 130 130 108 108],'Color','white','LineWidth',3);
h2=line([40 40 96 96 40],[192 216 216 192 192],'Color','white','LineWidth',3);
h3=line([89 89 112 112 89],[87 112 112 87 87],'Color','white','LineWidth',3);
h4=line([77 77 99 99 122 122 145 145 77],[140 183 183 168 168 183 183 140 140],'Color','white','LineWidth',3);
h5=line([93 93 190 190 93],[5 53 53 5 5],'Color','white','LineWidth',3);
h6=line([154 154 225 225 154],[144 200 200 144 144],'Color','white','LineWidth',3);
h7=line([350 350 426 426 350],[60 101 101 60 60],'Color','white','LineWidth',3);
h8=line([219 219 200 200 219 219 238 238 219],[36 46 46 67 67 77 77 36 36],'Color','white','LineWidth',3);
h9=line([267 267 293 293 267 267 338 338 267],[139 160 160 175 175 206 206 139 139],'Color','white','LineWidth',3);
h10=line([358 358 439 439 358],[127 204 204 127 127],'Color','white','LineWidth',3);
h11=line([53 39 55 77 66 53],[4 23 37 14 4 4],'Color','white','LineWidth',3);
h12=line([251 251 312 312 251],[51 113 91 41 51],'Color','white','LineWidth',3,'LineStyle','--');

Exposure=flip(Exposure)';



%%% TEMP (make user inputs)
start_node=[15,15,30];
goal_node=[x_source,y_source];
%goal_node=[40,90];
r=7;
c=2;
rpms=[30 30];
wheelRad=1;
L=5;
weight_exposure=0.9;
weight_dist=0.1;
%%% TEMP (make user inputs)

Obstacles=[h1 h2 h3 h4 h5 h6 h7 h8 h9 h10 h11 h12];
startInObstacle = obstacleCheckRigid(Obstacles,start_node,r,c);
goalInObstacle = obstacleCheckRigid(Obstacles,goal_node,r,c);

if or(startInObstacle==1,or(or(start_node(1)>xmax,start_node(1)<xmin),or(start_node(2)>ymax,start_node(2)<ymin)))
    outside_obs_start=1;
    while or(outside_obs_start==1,or(or(start_node(1)>xmax,start_node(1)<xmin),or(start_node(2)>ymax,start_node(2)<ymin)))
        % Display message if start node falls outside of action space 
        fprintf('\n');
        disp('INAVLID START NODE LOCATION!');
        fprintf('\n');
        prompt = 'Enter new x,y STARTING node location (e.g. [0,0,30]), relative to bottom left corner of action space: ';
        start_node = input(prompt);
        outside_obs_start = obstacleCheckRigid(Obstacles,start_node,r,c);
    end
end

if or(goalInObstacle==1,or(or(goal_node(1)>xmax,goal_node(1)<xmin),or(goal_node(2)>ymax,goal_node(2)<ymin)))
    outside_obs_goal=1;
    while or(outside_obs_goal==1,or(or(goal_node(1)>xmax,goal_node(1)<xmin),or(goal_node(2)>ymax,goal_node(2)<ymin)))
        % Display message if goal node falls outside of action space 
        fprintf('\n');
        disp('INAVLID GOAL NODE LOCATION!');
        fprintf('\n');
        prompt = 'Enter new x,y GOAL node location (e.g. [10,9]), relative to bottom left corner of action space: ';
        goal_node = input(prompt);
        outside_obs_goal = obstacleCheckRigid(Obstacles,goal_node,r,c);
    end
end

% Start program run timer
tic

% Plot yellow circle around goal node, representing distance
% threshold/margin
th = 0:pi/50:2*pi;
x_circle = 4 * cos(th) + goal_node(1);
y_circle = 4 * sin(th) + goal_node(2);
plot(x_circle, y_circle,'k-','LineWidth',3);
%fill(x_circle, y_circle, 'y');
drawnow

% Plot start and end point
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');
uistack(fig,'top');

start_node(3)=start_node(3)*(pi/180);
% Initialize start node info
Nodes(1).x=start_node(1);
Nodes(1).y=start_node(2);
Nodes(1).Explored=0;
Nodes(1).ParentID=0;
Nodes(1).Theta=wrapTo2Pi(start_node(3));
Nodes(1).ID=1;
Nodes(1).Cost2Go=sqrt((abs(start_node(1)-goal_node(1))^2)+(abs(start_node(2)-goal_node(2))^2))+Exposure(Nodes(1).x,Nodes(1).y);
Nodes(1).TotalCost=Nodes(1).Cost2Go+Exposure(start_node(1),start_node(2));
Nodes(1).interpsX=0;
Nodes(1).interpsY=0;
Nodes(1).LeftRPM=0;
Nodes(1).RightRPM=0;

% Convert angle to 0-2*pi scale
start_node(3)=wrapTo2Pi(start_node(3));

% Toggle that states goal node has not been explored
goal_node_explored=0;

% Initialize node ID counter and Parent ID counter
i=1;
ParentIdx=1;

rpm_vals{1}=[0,rpms(1)];
rpm_vals{2}=[rpms(1),0];
rpm_vals{3}=[rpms(1),rpms(1)];
rpm_vals{4}=[0,rpms(2)];
rpm_vals{5}=[rpms(2),0];
rpm_vals{6}=[rpms(2),rpms(2)];
rpm_vals{7}=[rpms(1),rpms(2)];
rpm_vals{8}=[rpms(2),rpms(1)];
n=length(rpm_vals);

Xs=[]; Ys=[];
xx=start_node(1); yy=start_node(2);
startAngle=start_node(3);
firstIteration=1;
T=[];


% While goal node HAS NOT BEEN EXPLORED, perform RRT
while goal_node_explored==0
    cost_ref=1000000000;
    if firstIteration==1
        firstIteration=0;
        firstIteration1=1;
        xmin1=xmin;
        xmax1=xmax;
        randX=xmin1+rand(1,1)*(xmax1-xmin1);
        ymin1=ymin;
        ymax1=ymax;
        randY=ymin1+rand(1,1)*(ymax1-ymin1);
        h_tmp=plot(randX,randY,'ro','MarkerSize',10,'MarkerFaceColor','r');
    else      
        delete(h_tmp); delete(h_tmp2);
        % Sort unexplored nodes by lowest total cost
        T = struct2table(Nodes); % convert the struct array to a table
        sortedT = sortrows(T, 'TotalCost'); % sort the table by 'TotalCost'
        toDelete = sortedT.Explored == 1;
        sortedT(toDelete,:) = [];
        sortedS = table2struct(sortedT); % change it back to struct array if necessary
        % Extract first node in the sorted queue
        if isempty(sortedS)==1
            fprintf('\n\n PROGRAM TERMINATED! NO FURTHER ACTIONS POSSIBLE! (interference w/obstacles or map boundary)')
            return 
        else
        end
        
        %xx=sortedS(1).x; yy=sortedS(1).y; 
        %ParentIdx=sortedS(1).ID; 
        %startAngle=sortedS(1).Theta;
        %isExplored=sortedS(1).Explored;
        
        % Generate random x and y coordinates within map boundary
        xmin1=xmin;
        xmax1=xmax;
        randX=xmin1+rand(1,1)*(xmax1-xmin1);
        ymin1=ymin;
        ymax1=ymax;
        randY=ymin1+rand(1,1)*(ymax1-ymin1);
        h_tmp=plot(randX,randY,'ro','MarkerSize',10,'MarkerFaceColor','r');
 
        
        Xs=[Nodes.x];
        Ys=[Nodes.y];
        IDs=[Nodes.ID];
        Angless=[Nodes.Theta];
        EXPLORED=[Nodes.Explored];
        dist_node=sqrt(((Xs-randX).^2)+((Ys-randY).^2));
        [MIN,MIN_IDX]=min(dist_node);
        xx=Xs(MIN_IDX);
        yy=Ys(MIN_IDX);
        ParentIdx=IDs(MIN_IDX);
        startAngle=Angless(MIN_IDX);
        isExplored=EXPLORED(MIN_IDX);
        
        
    end
    kk=1;
    for k=1:1:length(rpm_vals)
        % Generate new node
        if goal_node_explored==0
        rpmLeft=rpm_vals{k}(1);
        rpmRight=rpm_vals{k}(2);
        [newX,newY,newTheta,interpX,interpY,runningDist] = changeAngle(xx,yy,startAngle,rpmLeft,rpmRight,wheelRad,L);
        
        if firstIteration1==1
            isExplored=0;
            firstIteration1=0;
        else
        end
        
        % Check if new node is in any of the obstacles
        outsideObstaclesANDBorder=obstacleCheckRigid(Obstacles,[newX,newY],r,c);
        
        checker=0; checker1=0; checker2=0; checker3=0; 
        for iiii=1:1:length(Obstacles)
            h=Obstacles(iiii);
            In = inpolygon(interpX,interpY,h.XData+r+c,h.YData+r+c);
            In1= inpolygon(interpX,interpY,h.XData+r+c,h.YData-r-c);
            In2= inpolygon(interpX,interpY,h.XData-r-c,h.YData+r+c);
            In3= inpolygon(interpX,interpY,h.XData-r-c,h.YData-r-c);
            checker=checker+sum(In);
            checker1=checker1+sum(In1);
            checker2=checker2+sum(In2);
            checker3=checker3+sum(In3);
        end
        if or(or(or(checker>=1,checker1>=1),or(checker2>=1,checker3>=1)),or(or(newX>xmax,newX<xmin),or(newY>ymax,newY<ymin)))==1
            interpolatedSegmentsInBorder=1;
        else
            interpolatedSegmentsInBorder=0;
        end
       
        Anglee=wrapTo2Pi(newTheta);
                    
        % Check if new node is within 0.1 distance of any other node
        if isempty(Xs)==1
            num=0;
        else
            Xstmp=Xs-newX; Ystmp=Ys-newY;
            num=sum(and(abs(Xstmp)<=.1,abs(Ystmp)<=.1));
        end
        
        if and(and(newX>=xmin,newY>=ymin),num==0)                

            if and(and(and(outsideObstaclesANDBorder==0,interpolatedSegmentsInBorder==0),or(goal_node_explored==0,and(and(newX>=xmin,newX<=xmax),and(newY>=ymin,newY<=ymax))))==1,num==0)==1                
                cost2go=sqrt((abs(newX-x_source)^2)+(abs(newY-y_source)^2));
                sumCost=(weight_exposure*Exposure(round(newX),round(newY)))+(weight_dist*cost2go);
                Total_Cost(kk)=sumCost;
                Anglees(kk)=Anglee;
                x_vals(kk)=newX;
                y_vals(kk)=newY;
                interpXs{kk}=interpX;
                interpYs{kk}=interpY;
                rpmLefts(kk)=rpmLeft;
                rpmRights(kk)=rpmRight;
                
                kk=kk+1;
                
            else
            end
        else
        end
        else
        end
    end
    
    %Anglee=wrapTo2Pi(startAngle);
    
    if isempty(Total_Cost)==0
        [sumCost,min_idx]=min(Total_Cost);
        newX=x_vals(min_idx);
        newY=y_vals(min_idx);
        interpX=interpXs{min_idx};
        interpY=interpYs{min_idx};
        Anglee=Anglees(min_idx);
        rpmLeft=rpmLefts(min_idx);
        rpmRight=rpmRights(min_idx);
        h_tmp2=line([randX xx],[randY yy],'Color','red','LineStyle','--','LineWidth',1);

        
        closeEnough=sqrt((goal_node(1)-newX)^2+(goal_node(2)-newY)^2)<5;
        if closeEnough==1
            goal_node_explored=1;
        else
        end
        Total_Cost=[];
        Anglees=[]; 
        x_vals=[]; 
        y_vals=[]; 
        interpXs={}; 
        interpYs={}; 
        rpmLefts=[]; 
        rpmRights=[];

        i=i+1;
        drawnow
        xxx = interpX;
        yyy = interpY;
        % Cubic spline data interpolation
        tt = 1:numel(xxx);
        xy = [xxx;yyy];
        pp = spline(tt,xy);
        tInterp = linspace(1,numel(xxx));
        xyInterp = ppval(pp, tInterp);
        plot(xyInterp(1,:),xyInterp(2,:),'b-');
        Nodes(i).x=newX;
        Nodes(i).y=newY;
        Nodes(i).ParentID=ParentIdx;
        Nodes(i).ID=i;
        Nodes(i).Theta=Anglee;
        Nodes(i).Cost2Go=cost2go;
        Nodes(i).TotalCost=sumCost;
        Nodes(i).Explored=0;
        Nodes(i).interpsX=interpX;
        Nodes(i).interpsY=interpY;
        Nodes(i).LeftRPM=rpmLeft;
        Nodes(i).RightRPM=rpmRight;
        %XS=[XS newX]; YS=[YS newY];

        % Mark node as "Explored"
        Nodes(ParentIdx).Explored=1;
        
    else
    end
end

% Backtrack to find optimal path, and plot it with a red line
backTrackingFinished=0;
k=0;
k_tmp=0;

T = struct2table(Nodes); % convert the struct array to a table
toDelete = T.Explored == 0;
T(toDelete,:) = [];

node_idx=Nodes(end).ID;
delete(h_tmp); delete(h_tmp2);
while backTrackingFinished==0
    k=k+1;
    k_tmp=k_tmp+1;
    xxx = Nodes(node_idx).interpsX;
    yyy = Nodes(node_idx).interpsY;
    if or(and(xxx==0,yyy==0)==1,node_idx==1)
    else
        % Cubic spline data interpolation
        tt = 1:numel(xxx);
        xy = [xxx;yyy];
        pp = spline(tt,xy);
        tInterp = linspace(1,numel(xxx));
        xyInterp = ppval(pp, tInterp);
        plot(xyInterp(1,:),xyInterp(2,:),'g-','LineWidth',2);
    end
                    
    xVals(k)=Nodes(node_idx).x;
    yVals(k)=Nodes(node_idx).y;
    
    xVals_tmp(k_tmp)=Nodes(node_idx).x;
    yVals_tmp(k_tmp)=Nodes(node_idx).y;
    
    if length(xxx)<=1
    else
        for iii=1:1:length(xxx)
            k_tmp=k_tmp+1;
            xVals_tmp(k_tmp)=xxx(iii);
            yVals_tmp(k_tmp)=yyy(iii);
        end
    end
    
    plot(Nodes(node_idx).x,Nodes(node_idx).y,'go','MarkerFaceColor','green','MarkerSize',5);
    nodeNum(k)=k;
    nodeIndex(k)=node_idx;
    leftWheelVel(k)=Nodes(node_idx).LeftRPM;
    rightWheelVel(k)=Nodes(node_idx).RightRPM;
    orientation(k)=Nodes(node_idx).Theta;
    node_idx=Nodes(node_idx).ParentID;
    
    if and(xVals(k)==start_node(1),yVals(k)==start_node(2))
        backTrackingFinished=1;
    else
    end
end

drawnow
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');
drawnow
uistack(fig,'top');

X_Values=flip(xVals);
Y_Values=flip(yVals);

X_Values_tmp=flip(xVals_tmp);
Y_Values_tmp=flip(yVals_tmp);

Left_Wheel_RPMs=flip(leftWheelVel);
Right_Wheel_RPMs=flip(rightWheelVel);
Orientation=flip(orientation);

x_val_input="x_vals={";
y_val_input="y_vals={";
left_vels_input="left_vels={";
right_vels_input="right_vels={";
orientation_input="orientations={";
for p=1:1:length(X_Values)
    x_val_input=strcat(x_val_input,string(X_Values(p)));
    y_val_input=strcat(y_val_input,string(Y_Values(p)));
    left_vels_input=strcat(left_vels_input,string(Left_Wheel_RPMs(p)));
    right_vels_input=strcat(right_vels_input,string(Right_Wheel_RPMs(p)));
    orientation_input=strcat(orientation_input,string(Orientation(p)));
    if p==length(X_Values)
        x_val_input=strcat(x_val_input,"};");
        y_val_input=strcat(y_val_input,"};");
        left_vels_input=strcat(left_vels_input,"};");
        right_vels_input=strcat(right_vels_input,"};");
        orientation_input=strcat(orientation_input,"};");
    else
        x_val_input=strcat(x_val_input,",");
        y_val_input=strcat(y_val_input,",");
        left_vels_input=strcat(left_vels_input,",");
        right_vels_input=strcat(right_vels_input,",");
        orientation_input=strcat(orientation_input,",");
    end
end
disp(x_val_input)
disp(y_val_input)
disp(left_vels_input)
disp(right_vels_input)
disp(orientation_input)

% End program run timer
toc

course=[X_Values_tmp;Y_Values_tmp;zeros(1,length(X_Values_tmp))]';

figure; 
map_lats=[38.983417,38.985492,38.985482,38.983420,38.983417]; 
map_lons=[-76.945334,-76.945366,-76.940177,-76.940137,-76.945334];
geoplot(map_lats,map_lons,'k-','LineWidth',3);
geobasemap('topographic');
geolimits([min(map_lats) max(map_lats)],[min(map_lons) max(map_lons)]);

lla = enu2lla(course,[map_lats(1) map_lons(1) 0]);

course1=[h1.XData;h1.YData;zeros(1,length(h1.XData))]';
course2=[h2.XData;h2.YData;zeros(1,length(h2.XData))]';
course3=[h3.XData;h3.YData;zeros(1,length(h3.XData))]';
course4=[h4.XData;h4.YData;zeros(1,length(h4.XData))]';
course5=[h5.XData;h5.YData;zeros(1,length(h5.XData))]';
course6=[h6.XData;h6.YData;zeros(1,length(h6.XData))]';
course7=[h7.XData;h7.YData;zeros(1,length(h7.XData))]';
course8=[h8.XData;h8.YData;zeros(1,length(h8.XData))]';
course9=[h9.XData;h9.YData;zeros(1,length(h9.XData))]';
course10=[h10.XData;h10.YData;zeros(1,length(h10.XData))]';
course11=[h11.XData;h11.YData;zeros(1,length(h11.XData))]';
course12=[h12.XData;h12.YData;zeros(1,length(h12.XData))]';
lla1 = enu2lla(course1,[map_lats(1) map_lons(1) 0]);
lla2 = enu2lla(course2,[map_lats(1) map_lons(1) 0]);
lla3 = enu2lla(course3,[map_lats(1) map_lons(1) 0]);
lla4 = enu2lla(course4,[map_lats(1) map_lons(1) 0]);
lla5 = enu2lla(course5,[map_lats(1) map_lons(1) 0]);
lla6 = enu2lla(course6,[map_lats(1) map_lons(1) 0]);
lla7 = enu2lla(course7,[map_lats(1) map_lons(1) 0]);
lla8 = enu2lla(course8,[map_lats(1) map_lons(1) 0]);
lla9 = enu2lla(course9,[map_lats(1) map_lons(1) 0]);
lla10 = enu2lla(course10,[map_lats(1) map_lons(1) 0]);
lla11 = enu2lla(course11,[map_lats(1) map_lons(1) 0]);
lla12 = enu2lla(course12,[map_lats(1) map_lons(1) 0]);
hold on;
geoplot(lla1(:,1),lla1(:,2),'k-','LineWidth',2);
geoplot(lla2(:,1),lla2(:,2),'k-','LineWidth',2);
geoplot(lla3(:,1),lla3(:,2),'k-','LineWidth',2);
geoplot(lla4(:,1),lla4(:,2),'k-','LineWidth',2);
geoplot(lla5(:,1),lla5(:,2),'k-','LineWidth',2);
geoplot(lla6(:,1),lla6(:,2),'k-','LineWidth',2);
geoplot(lla7(:,1),lla7(:,2),'k-','LineWidth',2);
geoplot(lla8(:,1),lla8(:,2),'k-','LineWidth',2);
geoplot(lla9(:,1),lla9(:,2),'k-','LineWidth',2);
geoplot(lla10(:,1),lla10(:,2),'k-','LineWidth',2);
geoplot(lla11(:,1),lla11(:,2),'k-','LineWidth',2);
geoplot(lla12(:,1),lla12(:,2),'k--','LineWidth',2);

hold on;
for hhh=1:1:length(lla)
    geoplot(lla(hhh,1),lla(hhh,2),'bo','MarkerSize',3,'MarkerFaceColor','blue');
end
hold on;
geoplot(lla(1,1), lla(1,2),'mo','MarkerFaceColor','m','MarkerSize',6);
geoplot(lla(length(lla),1), lla(length(lla),2),'m^','MarkerFaceColor','m','MarkerSize',6);
title('Calculated Path Plan on College Park Map Projection (Scenario 1)','FontSize',16);
