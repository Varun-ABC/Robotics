close all 
clear all



%% 
% set up code:
L1 = 1.5;L2 = 1.5;L3 = .5;

P0=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
robot.L = [L1, L2, L3];
robot.P = [P0, L1*ex, L2*ex, L3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];
robot.q=[0;0;0];
radius = .01;
robot_rb=defineRobot(robot,radius);
load S_letter_path
N = size(Sls);
out_norm = zeros(N);
lambda = zeros(N(2) - 1,1);
nl=length(Sls);

for i = 1:size(Sls,2)-1
    vec_x = (Sls(1,i+1) - Sls(1,i));
    vec_y = (Sls(2,i+1) - Sls(2,i));
    norm_vec = norm([vec_x, vec_y]);
    lambda(i) = norm_vec;
    vec_x = vec_x / norm_vec;
    vec_y = vec_y / norm_vec;
    out_norm(1,i) = -vec_y; %x vector
    out_norm(2,i) = vec_x; %y vector

end
lambda = cumsum(lambda);
vec_x = (Sls(1,end-1) - Sls(1,end));
vec_y = (Sls(2,end-1) - Sls(2,end));
norm_vec = norm([vec_x, vec_y]);
vec_x = vec_x / norm_vec;
vec_y = vec_y / norm_vec;
out_norm(1,end) = vec_y; %x vector
out_norm(2,end) = -vec_x; %y vector
% divided by 10 to make the arrows smaller
[xT,yT] = setR0T(out_norm);

%end set up code
%% 

% P0 is at (0,0), thats why first element of robot.P is 0 0 0

% function Q = threelink_invkin_iterative(robot,a, q0, lin_tol, ang_tol, iter_max)


figure(3);
grid;axis([-1,3,-2,2]);axis('square');
hold on

a = .01;
q0 = [0 0 0]';
lin_tol = .01;
ang_tol = .01;
iter_max = 1000;
figure(4);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
grid;axis([-1,3,-2,2]);axis('square');
% for i=1:nl-1
for i=1:nl-1
    robot.T(1:3,1:4)=[xT(:,i), yT(:,i), ez, [Sls(:,i);0]];
    qsol_iter = threelink_invkin_iterative(robot, a, q0, lin_tol, ang_tol, iter_max);% <<<< you need to provide this
    robot.q= qsol_iter;
    % pause(.1);
    robot_rb=defineRobot(robot,radius);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
    hold on
    q0 = qsol_iter;
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


function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end


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