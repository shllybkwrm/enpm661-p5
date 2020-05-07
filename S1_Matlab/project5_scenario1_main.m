%%%
% Chris Wheatley, Shelly Bagchi
% ENPM661 Spring 2020
% Project #5 

%%%  (MAIN SCRIPT)  %%%

close all; 
clear all;

% Solicit input node configuration from user
fprintf('\n');
prompt = 'Enter x,y START node location in meters and starting orientation in deg. (e.g. [1,1,30]): ';
start_node = input(prompt);
fprintf('\n');

% Solicit goal node configuration from user
fprintf('\n');
prompt1 = 'Enter "1" (to evaluate goal/source in top LEFT of map) or "2" (to evaluate goal/source in top RIGHT of map): ';
source_info = input(prompt1);
fprintf('\n');

% Solicit obstacle clearance from user
fprintf('\n');
prompt3 = 'Enter obstacle clearance (meters): ';
c = input(prompt3);
fprintf('\n');

% Solicit wheel RPMs from user
fprintf('\n');
prompt3 = 'Enter wheel RPMs [rpm1, rpm2]: ';
rpms = input(prompt3);
fprintf('\n');

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
global x_source y_source
if source_info==1
    sourceInTopLeft=1;
    sourceInTopRight=0;
elseif source_info==2
    sourceInTopLeft=0;
    sourceInTopRight=1;
else
end

if sourceInTopLeft==1
    x_source=15;
    y_source=212;
elseif sourceInTopRight==1
    x_source=400;
    y_source=218;
else
end

global map_height map_width X Y
map_height=232; %m
map_width=450; %m
[X,Y]=meshgrid([1:1:map_width],[1:1:map_height]); %#ok<NBRAK>
X=flip(X);
Y=flip(Y);

% Define vertices of every obstacle
h1=line([15 15 36 36 56 56 15],[108 168 168 130 130 108 108],'Color','black','LineWidth',3);
h2=line([40 40 96 96 40],[192 216 216 192 192],'Color','black','LineWidth',3);
h3=line([89 89 112 112 89],[87 112 112 87 87],'Color','black','LineWidth',3);
h4=line([77 77 99 99 122 122 145 145 77],[140 183 183 168 168 183 183 140 140],'Color','black','LineWidth',3);
h5=line([93 93 190 190 93],[5 53 53 5 5],'Color','black','LineWidth',3);
h6=line([154 154 225 225 154],[144 200 200 144 144],'Color','black','LineWidth',3);
h7=line([350 350 426 426 350],[60 101 101 60 60],'Color','black','LineWidth',3);
h8=line([219 219 200 200 219 219 238 238 219],[36 46 46 67 67 77 77 36 36],'Color','black','LineWidth',3);
h9=line([267 267 293 293 267 267 338 338 267],[139 160 160 175 175 206 206 139 139],'Color','black','LineWidth',3);
h10=line([358 358 439 439 358],[127 204 204 127 127],'Color','black','LineWidth',3);
h11=line([53 39 55 77 66 53],[4 23 37 14 4 4],'Color','black','LineWidth',3);
h12=line([251 251 312 312 251],[51 113 91 41 51],'Color','black','LineWidth',3,'LineStyle','--');
fill(h1.XData,h1.YData,'w')
fill(h2.XData,h2.YData,'w')
fill(h3.XData,h3.YData,'w')
fill(h4.XData,h4.YData,'w')
fill(h5.XData,h5.YData,'w')
fill(h6.XData,h6.YData,'w')
fill(h7.XData,h7.YData,'w')
fill(h8.XData,h8.YData,'w')
fill(h9.XData,h9.YData,'w')
fill(h10.XData,h10.YData,'w')
fill(h11.XData,h11.YData,'w')
fill(h12.XData,h12.YData,'w')

% Find x,y centroid of each obstacle
[x1,y1]=calc_polygon_center(h1.XData,h1.YData);
[x2,y2]=calc_polygon_center(h2.XData,h2.YData);
[x3,y3]=calc_polygon_center(h3.XData,h3.YData);
[x4,y4]=calc_polygon_center(h4.XData,h4.YData);
[x5,y5]=calc_polygon_center(h5.XData,h5.YData);
[x6,y6]=calc_polygon_center(h6.XData,h6.YData);
[x7,y7]=calc_polygon_center(h7.XData,h7.YData);
[x8,y8]=calc_polygon_center(h8.XData,h8.YData);
[x9,y9]=calc_polygon_center(h9.XData,h9.YData);
[x10,y10]=calc_polygon_center(h10.XData,h10.YData);
[x11,y11]=calc_polygon_center(h11.XData,h11.YData);
[x12,y12]=calc_polygon_center(h12.XData,h12.YData);

% Find distances from every vertex to center
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

% Calc effective radius
rad1=1.1*max(dist1);
rad2=1.1*max(dist2);
rad3=1.1*max(dist3);
rad4=1.1*max(dist4);
rad5=1.1*max(dist5);
rad6=1.1*max(dist6);
rad7=1.1*max(dist7);
rad8=1.1*max(dist8);
rad9=1.1*max(dist9);
rad10=1.1*max(dist10);
rad11=1.1*max(dist11);
rad12=1.1*max(dist12);

cesium_137=[0.662,8.04e-6,  7.065e-6, 0.6,0.8];
concrete  =[0.662,8.062e-6, 6.083e-6, 0.6,0.8];
steel     =[0.323,2.875e-6, 2.953e-6, 0.3,0.4];
interaction_coefficient=extrapolation(cesium_137);
interaction_coefficient_concrete=extrapolation(concrete);
interaction_coefficient_steel=extrapolation(steel);
total_miu=interaction_coefficient*air_density; % 1/m
total_miu_concrete=interaction_coefficient_concrete*concrete_density; % 1/m
total_miu_steel=interaction_coefficient_steel*steel_density;
total_miu_steel_air=interaction_coefficient_steel*air_density;

R=zeros(map_height, map_width);
R1=zeros(map_height, map_width);
R2=zeros(map_height, map_width);
R3=zeros(map_height, map_width);
R4=zeros(map_height, map_width);
R5=zeros(map_height, map_width);
R6=zeros(map_height, map_width);
R7=zeros(map_height, map_width);
R8=zeros(map_height, map_width);
R9=zeros(map_height, map_width);
R10=zeros(map_height, map_width);
R11=zeros(map_height, map_width);
R12=zeros(map_height, map_width);

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

% Back up Rs to use with obs12 later
R1_12 = R1;
R2_12 = R2;
R3_12 = R3;
R4_12 = R4;
R5_12 = R5;
R6_12 = R6;
R7_12 = R7;
R8_12 = R8;
R9_12 = R9;
R10_12 = R10;
R11_12 = R11;

% % Main source attenuation
% Calculate parameters for bisecting line perpendicular to source
[a1,b1] = findLinePts(x1,y1);
[a2,b2] = findLinePts(x2,y2);
[a3,b3] = findLinePts(x3,y3);
[a4,b4] = findLinePts(x4,y4);
[a5,b5] = findLinePts(x5,y5);
[a6,b6] = findLinePts(x6,y6);
[a7,b7] = findLinePts(x7,y7);
[a8,b8] = findLinePts(x8,y8);
[a9,b9] = findLinePts(x9,y9);
[a10,b10] = findLinePts(x10,y10);
[a11,b11] = findLinePts(x11,y11);

% Find boolean matrix for each obstacle
isLeft1 = isLeft(a1,b1);
isLeft2 = isLeft(a2,b2);
isLeft3 = isLeft(a3,b3);
isLeft4 = isLeft(a4,b4);
isLeft5 = isLeft(a5,b5);
isLeft6 = isLeft(a6,b6);
isLeft7 = isLeft(a7,b7);
isLeft8 = isLeft(a8,b8);
isLeft9 = isLeft(a9,b9);
isLeft10 = isLeft(a10,b10);
isLeft11 = isLeft(a11,b11);

% Find direction of source from each obstacle
% 1=left, 0=right
sourceDir1 = isLeft1(ymax-y_source, x_source);
sourceDir2 = isLeft2(ymax-y_source, x_source);
sourceDir3 = isLeft3(ymax-y_source, x_source);
sourceDir4 = isLeft4(ymax-y_source, x_source);
sourceDir5 = isLeft5(ymax-y_source, x_source);
sourceDir6 = isLeft6(ymax-y_source, x_source);
sourceDir7 = isLeft7(ymax-y_source, x_source);
sourceDir8 = isLeft8(ymax-y_source, x_source);
sourceDir9 = isLeft9(ymax-y_source, x_source);
sourceDir10 = isLeft10(ymax-y_source, x_source);
sourceDir11 = isLeft11(ymax-y_source, x_source);

% Zero out values on source side of obstacle
R1(isLeft1==sourceDir1)=0;
R2(isLeft2==sourceDir2)=0;
R3(isLeft3==sourceDir3)=0;
R4(isLeft4==sourceDir4)=0;
R5(isLeft5==sourceDir5)=0;
R6(isLeft6==sourceDir6)=0;
R7(isLeft7==sourceDir7)=0;
R8(isLeft8==sourceDir8)=0;
R9(isLeft9==sourceDir9)=0;
R10(isLeft10==sourceDir10)=0;
R11(isLeft11==sourceDir11)=0;

% Threshold at attenuation "radius"
R1(R1>rad1)=0;
R2(R2>rad2)=0;
R3(R3>rad3)=0;
R4(R4>rad4)=0;
R5(R5>rad5)=0;
R6(R6>rad6)=0;
R7(R7>rad7)=0;
R8(R8>rad8)=0;
R9(R9>rad9)=0;
R10(R10>rad10)=0;
R11(R11>rad11)=0;

% % Obstacle 12 attenuation
% Calculate parameters for bisecting line perpendicular to 12
[a1,b1] = findLinePts(x1,y1, x12,y12);
[a2,b2] = findLinePts(x2,y2, x12,y12);
[a3,b3] = findLinePts(x3,y3, x12,y12);
[a4,b4] = findLinePts(x4,y4, x12,y12);
[a5,b5] = findLinePts(x5,y5, x12,y12);
[a6,b6] = findLinePts(x6,y6, x12,y12);
[a7,b7] = findLinePts(x7,y7, x12,y12);
[a8,b8] = findLinePts(x8,y8, x12,y12);
[a9,b9] = findLinePts(x9,y9, x12,y12);
[a10,b10] = findLinePts(x10,y10, x12,y12);
[a11,b11] = findLinePts(x11,y11, x12,y12);

% Find boolean matrix for each obstacle
isLeft1 = isLeft(a1,b1);
isLeft2 = isLeft(a2,b2);
isLeft3 = isLeft(a3,b3);
isLeft4 = isLeft(a4,b4);
isLeft5 = isLeft(a5,b5);
isLeft6 = isLeft(a6,b6);
isLeft7 = isLeft(a7,b7);
isLeft8 = isLeft(a8,b8);
isLeft9 = isLeft(a9,b9);
isLeft10 = isLeft(a10,b10);
isLeft11 = isLeft(a11,b11);

% Zero out values on source side of obstacle
R1_12(isLeft1==0)=0;
R2_12(isLeft2==0)=0;
R3_12(isLeft3==0)=0;
R4_12(isLeft4==0)=0;
R5_12(isLeft5==0)=0;
R6_12(isLeft6==0)=0;
R7_12(isLeft7==1)=0;
R8_12(isLeft8==0)=0;
R9_12(isLeft9==1)=0;
R10_12(isLeft10==1)=0;
R11_12(isLeft11==0)=0;

% Threshold at attenuation "radius"
R1_12(R1_12>rad1)=0;
R2_12(R2_12>rad2)=0;
R3_12(R3_12>rad3)=0;
R4_12(R4_12>rad4)=0;
R5_12(R5_12>rad5)=0;
R6_12(R6_12>rad6)=0;
R7_12(R7_12>rad7)=0;
R8_12(R8_12>rad8)=0;
R9_12(R9_12>rad9)=0;
R10_12(R10_12>rad10)=0;
R11_12(R11_12>rad11)=0;

% Calculate exposures and attenuations
source_strength=(9.45601235e17)./(4.*pi.*R.^2);
source_strength_steel = (9.45601235e17)./(4.*pi.*R12.^2);

% Attenuation of original source only
attenuation_at_R =exp(-total_miu.*abs(R));

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
attenuation_at_R12=exp(-total_miu_steel_air.*abs(R12));

concrete_attenuation_1=attenuation_at_R1+attenuation_at_R2+attenuation_at_R3+attenuation_at_R4+attenuation_at_R5+attenuation_at_R6+attenuation_at_R7+attenuation_at_R8+attenuation_at_R9+attenuation_at_R10+attenuation_at_R11;
% Remove some overlapping areas of 0s (which became 1 after exp) to prevent affecting entire map
concrete_attenuation_1(concrete_attenuation_1==10)=1;

% Attenuation of Obstacle 12 only
attenuation_at_R1=exp(-total_miu_concrete.*abs(R1_12));
attenuation_at_R2=exp(-total_miu_concrete.*abs(R2_12));
attenuation_at_R3=exp(-total_miu_concrete.*abs(R3_12));
attenuation_at_R4=exp(-total_miu_concrete.*abs(R4_12));
attenuation_at_R5=exp(-total_miu_concrete.*abs(R5_12));
attenuation_at_R6=exp(-total_miu_concrete.*abs(R6_12));
attenuation_at_R7=exp(-total_miu_concrete.*abs(R7_12));
attenuation_at_R8=exp(-total_miu_concrete.*abs(R8_12));
attenuation_at_R9=exp(-total_miu_concrete.*abs(R9_12));
attenuation_at_R10=exp(-total_miu_concrete.*abs(R10_12));
attenuation_at_R11=exp(-total_miu_concrete.*abs(R11_12));

concrete_attenuation_2=attenuation_at_R1+attenuation_at_R2+attenuation_at_R3+attenuation_at_R4+attenuation_at_R5+attenuation_at_R6+attenuation_at_R7+attenuation_at_R8+attenuation_at_R9+attenuation_at_R10+attenuation_at_R11;

response_function=1.835e-3*cesium_137(1)*interaction_coefficient;
response_function_steel=1.835e-3*steel(1)*interaction_coefficient_steel;
Exposure_rate=response_function.*source_strength.*attenuation_at_R;

Exposure_rate_steel=response_function_steel.*(0.25*source_strength_steel).*attenuation_at_R12;

Exposure=Exposure_rate.*concrete_attenuation_1;
Exposure_steel=Exposure_rate_steel.*concrete_attenuation_2;
Total_Exposure = Exposure + Exposure_steel;
% For display only - do not use these values for algorithm
Exposure_log = log(Total_Exposure);

hold on;
hh=surf(X,Y,Exposure_log,'FaceAlpha',.5,'EdgeAlpha',.5);
cc = colorbar;
cc.Label.String = "Log of Exposure Values";
view(2)
xlim([xmin xmax]); 
ylim([ymin ymax]);
hold on; 
z = get(hh,'ZData');
set(hh,'ZData',z-19)

Total_Exposure=flip(Total_Exposure)';
%Total_Exposure=flip(Exposure_log)';

goal_node=[x_source,y_source];
r=0.3;
wheelRad=.3;
L=0.6;
weight_exposure=0.000001;
weight_dist=0.999999;
% weight_exposure=0;
% weight_dist=1;

Obstacles=[h1 h2 h3 h4 h5 h6 h7 h8 h9 h10 h11 h12];
startInObstacle = obstacleCheckRigid(Obstacles,start_node,r,c);
goalInObstacle = obstacleCheckRigid(Obstacles,goal_node,r,c);

if or(startInObstacle==1,or(or(start_node(1)>xmax,start_node(1)<xmin),or(start_node(2)>ymax,start_node(2)<ymin)))
    outside_obs_start=1;
    while or(or(outside_obs_start==1,or(or(start_node(1)>xmax,start_node(1)<xmin),or(start_node(2)>ymax,start_node(2)<ymin))),or(start_node(1)==0,start_node(2)==0))
        % Display message if start node falls outside of action space 
        fprintf('\n');
        disp('INVALID START NODE LOCATION!');
        fprintf('\n');
        prompt = 'Enter new x,y STARTING node location (e.g. [0,0,30]), relative to bottom left corner of action space: ';
        start_node = input(prompt);
        outside_obs_start = obstacleCheckRigid(Obstacles,start_node,r,c);
    end
end

if or(source_info>2,source_info<1)==1
    while or(source_info>2,source_info<1)==1
        % Display message if goal node selection/toggle is either less than
        %   1 or greater than 2
        fprintf('\n');
        disp('INVALID SOURCE/GOAL NODE SELECTION! Please Enter "1" or "2"');
        fprintf('\n');
        prompt1 = 'Enter "1" (to evaluate goal/source in top LEFT of map) or "2" (to evaluate goal/source in top RIGHT of map): ';
        source_info = input(prompt1);      
    end
else
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
Nodes(1).Cost2Go=sqrt((abs(start_node(1)-goal_node(1))^2)+(abs(start_node(2)-goal_node(2))^2))+Total_Exposure(Nodes(1).x,Nodes(1).y);
Nodes(1).TotalCost=Nodes(1).Cost2Go+Total_Exposure(start_node(1),start_node(2));
Nodes(1).interpsX=0;
Nodes(1).interpsY=0;
Nodes(1).LeftRPM=0;
Nodes(1).RightRPM=0;
Nodes(1).Exposure=Total_Exposure(round(start_node(1)),round(start_node(2)));

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
        
        if sourceInTopLeft==1
             if sum(and(Xs<45,Ys>185))~=0
                xmin1=xmin;
                xmax1=45;
                randX=xmin1+rand(1,1)*(xmax1-xmin1);
                ymin1=185;
                ymax1=ymax;
                randY=ymin1+rand(1,1)*(ymax1-ymin1);
                h_tmp=plot(randX,randY,'ro','MarkerSize',10,'MarkerFaceColor','r');
             else
                 xmin1=xmin;
                 xmax1=xmax;
                 randX=xmin1+rand(1,1)*(xmax1-xmin1);
                 ymin1=ymin;
                 ymax1=ymax;
                 randY=ymin1+rand(1,1)*(ymax1-ymin1);
                 h_tmp=plot(randX,randY,'ro','MarkerSize',10,'MarkerFaceColor','r');
             end
        elseif sourceInTopRight==1
             if sum(and(Xs>350,Ys>209))~=0
                xmin1=350;                
                xmax1=xmax;
                randX=xmin1+rand(1,1)*(xmax1-xmin1);
                ymin1=209;
                ymax1=ymax;
                randY=ymin1+rand(1,1)*(ymax1-ymin1);
                h_tmp=plot(randX,randY,'ro','MarkerSize',10,'MarkerFaceColor','r');
             else
                 xmin1=xmin;
                 xmax1=xmax;
                 randX=xmin1+rand(1,1)*(xmax1-xmin1);
                 ymin1=ymin;
                 ymax1=ymax;
                 randY=ymin1+rand(1,1)*(ymax1-ymin1);
                 h_tmp=plot(randX,randY,'ro','MarkerSize',10,'MarkerFaceColor','r');
             end
        else
        end
 
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
        CloseToOuterBorder=or(or((newX-xmin)<=(r+c),(xmax-newX)<=(r+c)),or((newY-ymin)<=(r+c),(ymax-newY)<=(r+c)));
        CloseToOuterBorder_interps=sum(abs(xmax-interpX)<=(r+c))+sum(abs(xmin-interpX)<=(r+c))+sum(abs(ymax-interpY)<=(r+c))+sum(abs(ymin-interpY)<=(r+c));
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

            if and(and(and(and(outsideObstaclesANDBorder==0,interpolatedSegmentsInBorder==0),or(goal_node_explored==0,and(and(newX>=xmin,newX<=xmax),and(newY>=ymin,newY<=ymax))))==1,num==0),and(CloseToOuterBorder_interps==0,CloseToOuterBorder==0))==1                                
                %cost2go=sqrt((abs(newX-x_source)^2)+(abs(newY-y_source)^2));
                cost2go=sqrt((abs(newX-randX)^2)+(abs(newY-randY)^2));
                exposure=Total_Exposure(round(newX),round(newY));
                sumCost=(weight_exposure*exposure)+(weight_dist*cost2go);
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
        plot(xyInterp(1,:),xyInterp(2,:),'w-');
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
        Nodes(i).Exposure=exposure;
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
    
    plot(Nodes(node_idx).x,Nodes(node_idx).y,'g-','MarkerFaceColor','green','MarkerSize',5);
    nodeNum(k)=k;
    nodeIndex(k)=node_idx;
    leftWheelVel(k)=Nodes(node_idx).LeftRPM;
    rightWheelVel(k)=Nodes(node_idx).RightRPM;
    orientation(k)=Nodes(node_idx).Theta;
    exposure_vals(k)=Nodes(node_idx).Exposure; 
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

% Convert to VREP Coordinates
X_Values=flip(xVals)/10;
Y_Values=flip(yVals)/10;
X_Values=X_Values-25;
Y_Values=Y_Values-25;

X_Values_tmp=flip(xVals_tmp);
Y_Values_tmp=flip(yVals_tmp);

Left_Wheel_RPMs=flip(leftWheelVel);
Right_Wheel_RPMs=flip(rightWheelVel);
Orientation=flip(orientation);
Exposures=flip(exposure_vals);

x_val_input="x_vals={";
y_val_input="y_vals={";
left_vels_input="left_vels={";
right_vels_input="right_vels={";
orientation_input="orientations={";
exposures_input="exposures={";
for p=1:1:length(X_Values)
    x_val_input=strcat(x_val_input,string(X_Values(p)));
    y_val_input=strcat(y_val_input,string(Y_Values(p)));
    left_vels_input=strcat(left_vels_input,string(Left_Wheel_RPMs(p)));
    right_vels_input=strcat(right_vels_input,string(Right_Wheel_RPMs(p)));
    orientation_input=strcat(orientation_input,string(Orientation(p)));
    exposures_input=strcat(exposures_input,string(Exposures(p)));
    if p==length(X_Values)
        x_val_input=strcat(x_val_input,"};");
        y_val_input=strcat(y_val_input,"};");
        left_vels_input=strcat(left_vels_input,"};");
        right_vels_input=strcat(right_vels_input,"};");
        orientation_input=strcat(orientation_input,"};");
        exposures_input=strcat(exposures_input,"};");
    else
        x_val_input=strcat(x_val_input,",");
        y_val_input=strcat(y_val_input,",");
        left_vels_input=strcat(left_vels_input,",");
        right_vels_input=strcat(right_vels_input,",");
        orientation_input=strcat(orientation_input,",");
        exposures_input=strcat(exposures_input,",");
    end
end
disp(x_val_input)
disp(y_val_input)
disp(left_vels_input)
disp(right_vels_input)
disp(orientation_input)
disp(exposures_input)

% End program run timer
toc


% Generate selected path on a topographical map (showing actual College
%   Park campus infrastructure)
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


