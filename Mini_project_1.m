% Author: Varun Dhir
% Robotics 1: Mini- Project 1
% Date: 9/10/21
% Version 1.0

% initialize everything
clear all; close all;

% zr = small height 
zr=.1;

% Define robot as a planar circular shape
% radius = robot radius

radius_rob=.3;
robot = collisionCylinder(radius_rob,zr);

% position the robot in some initial configuration

p0_rob=[4;.5];th0_rob= deg2rad(90);
init_coll(robot,p0_rob, th0_rob)


% Person Init.

radius_per= .2;
person = collisionCylinder(radius_per,zr);

% position the person in some initial configuration

p0_pers=[1;.5];th0_pers= deg2rad(0);
init_coll(person,p0_pers, th0_pers)


% Table Init.

side_tab= .5; % square table
table = collisionBox(side_tab,side_tab,zr);

% position the table in some initial configuration

p0_tab=[2.5;2.5];th0_tab= deg2rad(45);
init_coll(table,p0_tab, th0_tab)

% Shelf Init.

x_shelf= .8;  
y_shelf = .3;
shelf = collisionBox(x_shelf,y_shelf ,zr);

% position the table in some initial configuration

p0_shelf=[1;4];th0_shelf = deg2rad(90);
% shelf.Pose(1:2,4)=p0_shelf;
% shelf.Pose(1:2,1:2)=rot2(th0_shelf);
init_coll(shelf,p0_shelf, th0_shelf)


% show objects and person in a 5 x 5 room
% h1 = figure handle
h1=showroom(1,5,5); % showroom function below
% hold the figure so you can plot over it
hold on 
show(person)
show(robot)
show(table)
show(shelf)

axis equal % so circles look like circles no matter screen size
rectangle('Position', [0 0 5 5]) % show the bounds of room 

%***********************************
% straight line paths
%***********************************
target_1_x = p0_shelf(1) + + y_shelf/2 + .1 + radius_rob;
target_1_y = p0_shelf(2); %from project description this could be anywhere 
% on the edge, for simplicity, I am choosing the center. 
target_1=[target_1_x;target_1_y];
% length of straight line path

N = 20;
delta = 5;

[path, end_point_1, L1, L2] = stright_line_path(N, delta, target_1, table, robot);


p0_rob=[4;.5];th0_rob= deg2rad(90);
init_coll(robot,p0_rob, th0_rob)
[path_neg, end_point_1_neg, L1_neg, L2_neg] = stright_line_path(N, -delta, target_1, table, robot);
L_neg=[L1_neg , L2_neg+L1_neg(end)];

if norm(end_point_1_neg - p0_rob) + norm(target_1 - end_point_1_neg)<...
    norm(end_point_1 - p0_rob) + norm(target_1 - end_point_1)
    path = path_neg;
    end_point_1 = end_point_1_neg;
    L1 = L1_neg;
    L2 = L2_neg;
end
L=[L1, L2+L1(end)];
delta_1 = end_point_1-p0_rob;
Lf1= norm(delta_1);

delta_2 = target_1-end_point_1;
Lf2= norm(delta_2);


target_2 = [1; 1.1];
[path_2, end_point_1_2, L1_2, L2_2] = stright_line_path(N, -delta, target_2, shelf, robot);
% L=[L1_2, L2_2+L1_2(end)];
L=[L L1_2, L2_2+L1_2(end) L2_2];

delta_3 = target_2-target_1;
Lf3= norm(delta_3);

theta_target_1 = deg2rad(180); 
% since the path to the first target is split into 2 parts, it will rotate
% part way on one leg and part way on the other leg
theta_L1 = theta_target_1 * (Lf1 / (Lf1 + Lf2)); % number
deg_theta_L1 = rad2deg(theta_L1);
theta_1 = (1-L1/Lf1).* th0_rob + L1/Lf1.*theta_L1;
theta_L2 = theta_target_1; % number
theta_2 = (1-L2/Lf2).* theta_1(end) + L2/Lf2.* theta_L2;
deg_theta_2 = rad2deg(theta_2);
theta_target_2 = deg2rad(-90); % number
theta_3 = (1-L1_2/Lf3).* theta_2(end) + L1_2/Lf3.* theta_target_2;
theta = [theta_1 theta_2 theta_3];

path = [path path_2];

M = zeros(length(L),1);
j = 0;
for i=1:length(L)
    init_coll(robot,path(:,i), theta(i));
    show_add_arrows(robot)
    title("Robot path and orientation")
    xlabel("Room (m)")
    ylabel("Room (m)")
end
%________________________________________________
% Part C
% index path with time (first for straight line paths)
%______________________________________________

% maximum velocity of the vehicle
umax=[2;2]; % m/s
wmax=1; % rad/ s
% we have 3 distint paths: path 1, past the table, path 2, to target 1,
% path 3 to target 2

% path 1: p0 => end_point_1
% path 2: end_point_1 => target_1
% path 3: target_1 => target_2

% max speed for segment 1
path1_prime=(delta_1)/Lf1;
l1dotmax= min(abs(umax./path1_prime));
T1 = Lf1/l1dotmax;
% max speed of segment 2

path2_prime=(delta_2)/Lf2;
l2dotmax=min(abs(umax./path2_prime));
T2 = Lf2/l2dotmax;
% max speed of segment 3
path3_prime = (delta_3)/Lf3;
l3dotmax=min(abs(umax./path3_prime));
T3 = Lf3/l2dotmax;
if (theta_L1 - th0_rob)/ T1 > wmax
    T1 = abs((theta_L1 - th0_rob) / wmax);
    l1dotmax = Lf1 / T1;
end
if (theta_L2 - theta_L1)/ T2 > wmax
    T2 = abs((theta_L2 - theta_L1) / wmax);
    l2dotmax = Lf2 / T2;
end
if (theta_target_2 - theta_L2)/ T3 > wmax
    T3 = abs((theta_target_2 - theta_L2) / wmax);
    l3dotmax = Lf3 / T3;
end

w1 = (theta_L1 - th0_rob)/ T1; 
w2 = (theta_L2 - theta_L1)/ T2; 
w3 = (theta_target_2 - theta_L2)/ T3;
%----------------------------------------------------------%
% Motion Generation
%----------------------------------------------------------%

% use vehicle kinematics to generate motion
% note that lambda_dot is chosen so that velocity limits are not violated
ts=.05; % 0.05 second sampling period
t1=(0:ts:T1);
NT1=length(t1); %number of elements in T1
pt1=zeros(2,NT1);
pt1(:,1)=p0_rob;
theta_time_1 = zeros(1,NT1);
theta_time_1(1) = th0_rob;
% use simple finite difference to propagate robot motion
% assuming you can control the robot velocity

% function Xdot = owmr(X,U,Umax,Umin)
%     Xdot = max(min(U,Umax),Umin);
% end

for i=1:NT1-1
    pt1(:,i+1)=pt1(:,i)+ts*owmr(pt1(:,i),path1_prime*l1dotmax,umax,-umax);
    theta_time_1(i+1) = theta_time_1(i) + ts * w1;
end

% second segment
t2 = (T1:ts:T1+T2);
NT2 = length(t2);
pt2 = zeros(2,NT2);
pt2(:,1)=pt1(:,NT1);
theta_time_2 = zeros(1,NT2);
theta_time_2(1) = theta_time_1(end);

for i=1:NT2-1
    pt2(:,i+1)=pt2(:,i)+ts*owmr(pt2(:,i),path2_prime*l2dotmax,umax,-umax);
    theta_time_2(i+1) = theta_time_2(i) + ts * w2;
end

% third segment
t3=(T2+T1:ts:T1+T2+T3);
NT3=length(t3);
pt3=zeros(2,NT3);
pt3(:,1)=pt2(:,NT3);
theta_time_3 = zeros(1,NT3);
theta_time_3(1) = theta_time_2(end);

for i=1:NT3-1
    pt3(:,i+1)=pt3(:,i)+ts*owmr(pt3(:,i),path3_prime*l3dotmax,umax,-umax);
    theta_time_3(i+1) = theta_time_3(i) + ts * w3;
end
% combine motion
t=[t1 t2(2:NT2) t3(2:NT3)];
pt=[pt1 pt2(:, 2:NT2) pt3(:, 2:NT3)];
angle = [theta_time_1, theta_time_2(2:NT2), theta_time_3(2:NT3)];
% calculate path length
lt=[0 cumsum(vecnorm(diff(pt')'))];
% motion vs. time 
figure(11);
plot(t,pt,'linewidth',2);
legend('x','y');
xlabel('time (sec)');
ylabel('robot position (m)');
% lambda vs. time (piecewise linear, with a kink at transition)
figure(12);
plot(t,lt,'linewidth',2);
xlabel('time (sec)');
ylabel('path length (lambda) (m)');

figure(13);
plot(t, angle, 'linewidth',2);
xlabel('time (sec)');
ylabel('Angle (rad)');

figure(14);
xlabel('time (sec)');
ylabel('Angle (deg)');
plot(t, rad2deg(angle), 'linewidth',2);

% _____________________________________________________________%
% Function Definitions:


function out = show_add_arrows(robot)
    theta = atan2(robot.Pose(1,1),robot.Pose(2,1));
    Ax = show(robot);
    quiver(Ax, robot.Pose(1,4), robot.Pose(2,4), -cos(theta), -sin(theta), 'color', [1, 0, 0]);
    hold on
    quiver(Ax, robot.Pose(1,4), robot.Pose(2,4),cos(theta + deg2rad(90)), sin(theta + deg2rad(90)), 'color', [0, 1, 0]);
    hold on 
end

% can only handle one object at the moment
% can only handle 2 segmements at the moment
% N: Number of elements in the first lambda vector (first line segments)
% to keep delta lambda the same across all line segments
% delta: step of angle, larger delta runs faster but is more corse
% target: Final Target
% stat_object: static object
% mov_obj: mobile robot
% line_segments: amount of line segments in path, currently can only handle
% 2

function [path_opt, end_point_1, L1, l2]= stright_line_path(N, delta, target, stat_object, mov_obj)
    p0 = mov_obj.Pose(1:2,4);
    p_obj = stat_object.Pose(1:2,4);
    path2 = 0;
    coll = true;
    
    if delta > 0
        max_theta = 90;
    else 
        max_theta = -90;
    end
    Lf1_straight= norm(target-p0);
    % end_point_1
    % generate a lambda vector
    L1_stright=(0:Lf1_straight/N:Lf1_straight);
    % generate the path based on lambda
    path_stright = p0*(1-L1_stright/Lf1_straight) + target*L1_stright/Lf1_straight;
    % # of elements in L1
    n1=length(L1_stright); 
    for i=1:n1
        mov_obj.Pose(1:2,4)=path_stright(:,i);
        if checkCollision(stat_object, mov_obj) == 1
            coll = false;
            break
        end
    end
    if coll == true
        path_opt = path_stright;
        end_point_1 = target;
        L1 = L1_stright;
        l2 = [];
        return 
    end
    for theta = 0:delta:max_theta
        coll = true;
        theta_rad = theta*pi/180;
        R = [cos(theta_rad) -sin(theta_rad) ;sin(theta_rad) cos(theta_rad)] ;
        end_point_1 = R*p_obj;
        del_y = target(2) - p_obj(2);
        if del_y < 0
            end_point_1(2) = p_obj(2)-.2;
        else
            end_point_1(2) = p_obj(2)+.2;
        end
        Lf1= norm(end_point_1-p0);
        % end_point_1
        % generate a lambda vector
        L1=(0:Lf1/N:Lf1);
        % generate the path based on lambda
        path1 = p0*(1-L1/Lf1) + end_point_1*L1/Lf1;
        % # of elements in L1
        n1=length(L1); 
        
        for i=1:n1
            mov_obj.Pose(1:2,4)=path1(:,i);
            if checkCollision(stat_object, mov_obj) == 1
                coll = false;
                break
            end
        end
        if coll == false
            continue
        end
        % generate second line segement
       
        lf2=norm(target-end_point_1);
        l2=(lf2/N:lf2/N:lf2);
        n2=length(l2);
        path2 = end_point_1*(1-l2/lf2) + target*l2/lf2;
        
        for i=1:n2
            mov_obj.Pose(1:2,4)=path2(:,i);
            if checkCollision(stat_object, mov_obj) == 1
                coll = false;
                break
            end
        end
        if coll == false
            continue
        end
        break % breaks it on the first path it finds
    end
    if path2 ~= 0
        path_opt = [path1 path2];
    else
        path_opt = path1;
        l2 = [];
    end
end


function out = init_coll(coll_obj,p0, th0)

coll_obj.Pose(1:2,4)=p0;
coll_obj.Pose(1:2,1:2)=rot2(th0);
end
%
% show room 
%  
% input:    fignum = figure number
%           xdim = horizontal dimension of room
%           ydim = vertical dimension of room
%
function h=showroom(fignum,xdim,ydim)
    h=figure(fignum);
    % adjust the view so you can see from above
    view(0,90)
    % adjust the figure range so you can see the 5x5 room
    axis([0 xdim 0 ydim]);
    grid;
end

% 2D rotation matrix
% 
% input = theta (angle)
% output = 2x2 SO(2) matrix
%
function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end

%
% kinematics of an omni-directional wheeled mobile robot
%
% input: 
%       X = current state (x, y, theta)
%       U = current input (xdot, ydot, thetadot)
% 
% output: 
%       Xdot = current velocity
%
function Xdot = owmr(X,U,Umax,Umin)
    Xdot = max(min(U,Umax),Umin);
end

%
% kinematics of an nonholonomic wheeled mobile robot
%
% input: 
%       X = current state (x, y, theta)
%       U = current input (u=forward velocity, w=turning angular velocity)
% 
% output: 
%       Xdot = current velocity
%
function Xdot = nwmr(X,U,Umax,Umin)
    Xdot = [cos(X(3)) 0;sin(X(3)) 0; 0 1]*max(min(U,Umax),Umin); 
end