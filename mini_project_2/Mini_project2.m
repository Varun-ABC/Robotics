% Robotics 1- Mini - Project 2
close all 
clear all

L1 = 1.5;L2 = 1.5;L3 = .5;
load S_letter_path
% P0 is at (0,0), thats why first element of robot.P is 0 0 0
P0=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
robot.L = [L1, L2, L3];
robot.P = [P0, L1*ex, L2*ex, L3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];
robot.q=[0;0;0];
radius = .01;
robot_rb=defineRobot(robot,radius);
%% Forward/ inverse kinimatics check
a = -pi;
b = pi;
q_check = (b-a).*rand(3,1) + a

robot.q = q_check;
[T,J] = forward_kin(robot);
robot.T = T;
Q_check = threelink_invkin_geometric(robot)
%%


figure(1);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
xlabel('x');ylabel('y');
hold on
% axis([0 3.5 -1.5 1.5]);
% xlim([0, 3.5]);
% ylim([-1.5, 1.5])
% axis equal;grid;
N = size(Sls);
out_norm = zeros(N);
lambda = zeros(N(2) - 1,1);

for i = 1:size(Sls,2)-1
    vec_x = (Sls(1,i+1) - Sls(1,i));
    vec_y = (Sls(2,i+1) - Sls(2,i));
    norm_vec = norm([vec_x, vec_y]);
    lambda(i) = norm_vec;
    vec_x = vec_x / norm_vec;
    vec_y = vec_y / norm_vec;
    out_norm(1,i) = -vec_y; %x vector
    out_norm(2,i) = vec_x; %y vector
    quiver(Sls(1,i), Sls(2,i), out_norm(1,i)/10, out_norm(2,i)/10); 
    % divided by 10 to make the arrows smaller
    hold on
end
lambda = cumsum(lambda);
vec_x = (Sls(1,end-1) - Sls(1,end));
vec_y = (Sls(2,end-1) - Sls(2,end));
norm_vec = norm([vec_x, vec_y]);
vec_x = vec_x / norm_vec;
vec_y = vec_y / norm_vec;
out_norm(1,end) = vec_y; %x vector
out_norm(2,end) = -vec_x; %y vector
quiver(Sls(1,end), Sls(2,end), out_norm(1,end)/10, out_norm(2,end)/10); 
% divided by 10 to make the arrows smaller
hold on

% P0 is at (0,0), thats why first element of robot.P is 0 0 0
P0=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
robot.L = [L1, L2, L3];
robot.P = [P0, L1*ex, L2*ex, L3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];
robot.q=[0;0;0];
radius = .01;
robot_rb=defineRobot(robot,radius);
% figure(10);
hold on;show(robot_rb,robot.q,'Collision','on');
view(0,90);
axis([0,4,-2,2]);axis('square');
xlabel('x-axis (m)');ylabel('y-axis (m)');
title('Planar RRR arm in zero configuration (q_1=q_2=q_3=0)')
hold off

[xT,yT] = setR0T(out_norm);

nl=length(Sls);
qsol1=zeros(3,nl-1);
qsol2=zeros(3,nl-1);
% qsol3=zeros(3,nl-1);
% qsol4=zeros(3,nl-1);
for i=1:nl-1
% for i=1:1
    robot.T(1:3,1:4)=[xT(:,i), yT(:,i), ez, [Sls(:,i);0]];
    % tstart(i)=tic;
    qsol_geo=threelink_invkin_geometric(robot);% <<<< you need to provide this
    % telapsed(i)=toc(tstart(i));
    qsol1(:,i)=qsol_geo(:,1);
    qsol2(:,i)=qsol_geo(:,2);
end

max_q_dot = [1,1,1];
lambda_dot = max_lambda_dot(robot, max_q_dot, qsol1);
j_vel = zeros(3,nl-2);
% Velocity for loop
for i=1:nl-1
    robot.q = qsol1(:,i);
    p1 = Sls(:,i);
    p2 = Sls(:,i+1);
    j_vel(:,i) = joint_vel(robot, lambda_dot, p1, p2);   
end
figure(2)
plot(lambda,j_vel(1,:))
hold on
plot(lambda,j_vel(2,:))
hold on
plot(lambda,j_vel(3,:))
hold on
legend("joint 1","joint 2","joint 3" )
title("Joint velocity as a function of Lambda")
xlabel("Lambda (m)")
ylabel("Joint Velocity")



figure(2);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
grid;axis([-1,3,-2,2]);axis('square');
% Plotting for loop
for i=1:3:nl-1
    robot.q=qsol1(:,i);%pause(.1);
    robot_rb=defineRobot(robot,radius);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end

figure(3);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
grid;axis([-1,3,-2,2]);axis('square');
% Plotting for loop
for i=1:3:nl-1
    robot.q=qsol2(:,i);%pause(.1);
    robot_rb=defineRobot(robot,radius);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end
grid;

% iterative solution

qsol1_iter=zeros(3,nl-1);
robot.q=[0;0;0];
q0 = [0,0,0]';
a = .01;
lin_tol = .001;
ang_tol = .001;
iter_max = 100;
for i=1:nl-1
% for i=1:3
    robot.T(1:3,1:4)=[xT(:,i), yT(:,i), ez, [Sls(:,i);0]];
    % function Q = threelink_invkin_iterative(robot,a, q0, lin_tol, ang_tol, iter_max)

    qsol_iter=threelink_invkin_iterative(robot, a, q0, lin_tol, ang_tol, iter_max);% <<<< you need to provide this
    % telapsed(i)=toc(tstart(i));
    qsol1_iter(:,i)=qsol_iter;
    q0 = qsol_iter;
end
figure(4);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
grid;axis([-1,3,-2,2]);axis('square');
for i=1:3:nl-1
    robot.q= qsol1_iter(:,i);pause(.1);
    robot_rb=defineRobot(robot,radius);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end
%%
%

function [T, J] = forward_kin(robot)
    Q = robot.q;
    L = robot.L;
    qt= sum(Q);
    Q_cum = cumsum(Q);
    R0t = rot2(qt);
    L1 = L(1);
    L2 = L(2);
    L3 = L(3);
    Xt = L(1) * cos(Q_cum(1)) + L(2) * cos(Q_cum(2)) + L(3) * cos(Q_cum(3));
    Yt = L(1) * sin(Q_cum(1)) + L(2) * sin(Q_cum(2)) + L(3) * sin(Q_cum(3));
    T = eye(4);
    T(1:2,1:2) = R0t;
    T(1,end) = Xt; 
    T(2,end) = Yt; 
    J = [1,1,1;...
        -L1 * sin(-Q_cum(1)) - L2 * sin(Q_cum(2)) - L3 * sin(Q_cum(3)), - L2 * sin(Q_cum(2)) - L3 * sin(Q_cum(3)),  - L3 * sin(Q_cum(3));...
        L1*cos(Q_cum(1)) + L2*cos(Q_cum(2)) + L3*cos(Q_cum(3)), L2*cos(Q_cum(2)) + L3*cos(Q_cum(3)), L3*cos(Q_cum(3))];
end
% T; end effector pose T
function Q = threelink_invkin_geometric(robot)
    % robot.T(1:3,1:4)=[xT(:,i), yT(:,i), ez, [Sls(:,i);0]];
    Target = robot.T;
    path_point = Target(1:3,4);
    px = path_point(1);
    py = path_point(2);
    xT = Target(1:3,1);
    cos_qt = xT(1);
    sin_qt = xT(2);
    qt =  atan2(sin_qt, cos_qt) - deg2rad(180);
    L = robot.L;
    P3x = px - L(3) * cos(qt);
    P3y = py - L(3) * sin(qt);
    p3 = [P3x, P3y];
    p0 = [0,0];
    C1 = [P3x, P3y];
    C2 = p0;
    [P2_1, P2_2] = inter_2_circ(C1, robot.L(1), C2, robot.L(2));
    k = [0, 0, 1];
    p1_p2_1 = P2_1 - p0;
    p1_p2_2 = P2_2 - p0;
    
    p2_p3_1 = p3 - P2_1;
    p2_p3_2 = p3 - P2_2;
    
    q1_1 = subprob0_1(k, [1,0,0], [p1_p2_1, 0]);
    q1_2 = subprob0_1(k, [1,0,0], [p1_p2_2, 0]);
    
    q2_1 = subprob0_1(k, [p1_p2_1, 0], [p2_p3_1, 0]);
    q2_2 = subprob0_1(k, [p1_p2_2, 0], [p2_p3_2, 0]);
    
    q3_1 = qt - (q1_1 + q2_1);
    q3_2 = qt - (q1_2 + q2_2);
    Q = [q1_1, q1_2; q2_1,q2_2 ; q3_1 , q3_2];
end


function Q = threelink_invkin_algebraic(robot)
    Target = robot.T;
    path_point = Target(1:3,4);
    px = path_point(1);
    py = path_point(2);
    xT = Target(1:3,1);
    cos_qt = xT(1);
    sin_qt = xT(2);
    qt =  atan2(sin_qt, cos_qt) - deg2rad(180);
    L = robot.L;
    P3x = px - L(3) * cos(qt);
    P3y = py - L(3) * sin(qt);
    P3x_P3y_sq = abs(P3x^2) + abs(P3y^2);
    cos_q2 = (P3x_P3y_sq - abs(L(1)^2) - abs(L(2)^2))/ (2 * L(1)*L(2));
    sin_q2_1 = (1-abs(cos_q2^2))^.5; % elbow down
    sin_q2_2 = -(1-abs(cos_q2^2))^.5; % elbow up

    q2_1 = atan2(sin_q2_1, cos_q2);
    q2_2 = -q2_1;
    sin_q1 = ((L(1)+ L(2)*cos_q2) * P3y - L(2)*sin_q2_1 * P3x)/ P3x_P3y_sq;
    cos_q1 = ((L(1)+ L(2)*cos_q2) * P3x + L(2)*sin_q2_1 * P3y)/ P3x_P3y_sq;
    sin_q1_2 = ((L(1)+ L(2)*cos_q2) * P3y - L(2)*sin_q2_2 * P3x)/ P3x_P3y_sq;
    cos_q1_2 = ((L(1)+ L(2)*cos_q2) * P3x + L(2)*sin_q2_2 * P3y)/ P3x_P3y_sq;
    
    q1_1 = atan2(sin_q1, cos_q1);
    q1_2 = atan2(sin_q1_2, cos_q1_2);
    q3_1 = qt - (q1_1 + q2_1);
    q3_2 = qt - (q1_2 + q2_2);
    Q = [q1_1, q1_2; q2_1,q2_2 ; q3_1 , q3_2];
end

function Q = threelink_invkin_iterative(robot,a, q0, lin_tol, ang_tol, iter_max)
    Target = robot.T;
    path_point = Target(1:3,4);
    px = path_point(1);
    py = path_point(2);
    xT = Target(1:3,1);
    cos_qt = xT(1);
    sin_qt = xT(2);
    qt =  atan2(sin_qt, cos_qt) - deg2rad(180);
    qk = q0;
    i =0 ;
    while i< iter_max
        i = i+1;
        robot.q = qk;
        [T_curr, J_curr] = forward_kin(robot);
        tool_point = T_curr(1:3,4);
        px_curr = tool_point(1);
        py_curr = tool_point(2);
        xT_curr = T_curr(1:3,1);
        cos_qt_curr = xT_curr(1);
        sin_qt_curr = xT_curr(2);
        qt_curr =  atan2(sin_qt_curr, cos_qt_curr);
        error = [qt, px, py]  - [qt_curr, px_curr, py_curr];
        qk = qk + a * pinv(J_curr)* error';
        error_abs = abs(error);
        if error_abs(1) < lin_tol
            if error_abs(2) <  lin_tol
                if error_abs(3) < ang_tol
                    break
                end 
            end
        end
    end
    Q = qk;
end


function Q = threelink_invkin_iterative_broken(robot,a, q0, lin_tol, ang_tol, iter_max)
    Target = robot.T;
    path_point = Target(1:3,4);
    px = path_point(1);
    py = path_point(2);
    xT = Target(1:3,1);
    cos_qt = xT(1);
    sin_qt = xT(2);
    acos(cos_qt)
    asin(sin_qt)
    qt =  atan2(sin_qt, cos_qt) - deg2rad(180);
    qk = q0;
    i =0;
    while i< iter_max
        i = i+1;
        robot.q = qk;
%         [T_curr, J_curr] = forward_kin(robot);
%         tool_point = T_curr(1:3,4);
%         px_curr = tool_point(1);
%         py_curr = tool_point(2);
%         xT_curr = T_curr(1:3,1);
%         cos_qt_curr = xT_curr(1);
%         sin_qt_curr = xT_curr(2);
%         qt_curr =  atan2(sin_qt_curr, cos_qt_curr);
%         error = [qt, py, px]  - [qt_curr, px_curr, py_curr];
%         qk = qk + a * transpose(J_curr)* error';
        [T_curr, J_curr] = forward_kin(robot);
        tool_point = T_curr(1:3,4);
        px_curr = tool_point(1);
        py_curr = tool_point(2);
        xT_curr = T_curr(1:3,1);
        cos_qt_curr = xT_curr(1);
        sin_qt_curr = xT_curr(2);
        qt_curr =  atan2(sin_qt_curr, cos_qt_curr);
        error = [qt, py, px]  - [qt_curr, px_curr, py_curr];
        qk = qk - a * transpose(J_curr)* error';
        error_abs = abs(error);

         if error_abs(1) < lin_tol
            if error_abs(2) <  lin_tol
                if error_abs(3) < ang_tol
                    break
                end 
            end
        end
    end
    Q = qk;
    
end


function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end

function [I1, I2] = inter_2_circ(C1, r1, C2, r2)
    if size(C1,1) > 1
        C1 = C1';
    end
    if size(C2,1) > 1
        C2 = C2';
    end
    center_dist = norm(C1-C2); % distance between circles
    cosA = (r2^2+center_dist^2-r1^2)/(2*r2*center_dist);
    u_AB = (C2 - C1)/center_dist; % unit vector from first to second center
    pu_AB = [u_AB(2), -u_AB(1)]; % perpendicular vector to unit vector
    A = C1 + u_AB * (r2*cosA);
    B = pu_AB * (r2*sqrt(1-cosA^2));
    I1 =  A + B;
    I2 = A - B;
    
end

function [xT,yT] = setR0T(out_norm)
    N = size(out_norm,2);
    p1 = [1 0 0];
    k = [0 0 1];
    xT = zeros(3, N);
    yT = zeros(3, N);
    for i = 1:N
        qt = subprob0_1(k, p1, [out_norm(:,i)' 0]);
        xT(:,i) = [cos(qt), sin(qt), 0]';
        yT(:,i) = [-sin(qt), cos(qt), 0]';
    end
end

function q=subprob0_1(k,p1,p2)

    if ((k'*p1)>sqrt(eps)|(k'*p2)>sqrt(eps))
      error('k must be perpendicular to p and q');
    end

    p1=p1/norm(p1);
    p2=p2/norm(p2);

    q=2*atan2(norm(p1-p2),norm(p1+p2));
    cross_P = k'*(cross(p1,p2));
    if cross_P(end,end)<0
      q=-q;
    end

end

function lambda_dot = max_lambda_dot(robot, max_q_dot, qsol)
    N = size(qsol,2);
    l_dot_lin = 100 * ones(1,N);
    for i = 1: N-2
        robot.q = qsol(:,i);
        mod = sign(qsol(:,i+1) - qsol(:,i));
        [T, J] = forward_kin(robot);
        vel_vec =  J * (mod'.*max_q_dot)';
        b = sqrt(abs(vel_vec(1))^2 + abs(vel_vec(2))^2);
        if b ~= 0
            l_dot_lin(i) = b;
        end
    end
    lambda_dot = min(l_dot_lin);
end

function q_dot = joint_vel(robot, lambda_dot, p1, p2) 
    p1x = p1(1);
    p1y = p1(2);
    p2x = p2(1);
    p2y = p2(2);
    dy = p2y - p1y;
    dx = p2x - p1x;
    y_dot = lambda_dot * dy / (abs(dy) + abs(dx));
    x_dot = sqrt(lambda_dot^2 - abs(y_dot)^2) * sign(dx);
    % x = lsqlin(C,d,A,b,Aeq,beq,lb,ub)
    [~, J] = forward_kin(robot);
    % C is Jac
    J_lin_vel = J(1:2,1:3);
    lin_vel = [x_dot , y_dot]';
    lb = [-1,-1,-1]';
    ub = [1,1,1]';
    q_dot = lsqlin(J_lin_vel, lin_vel, zeros(size(J_lin_vel)),zeros(size(lin_vel)), zeros(size(J_lin_vel)),zeros(size(lin_vel)),lb,ub);
end







