close all
clear all
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

px = 1;
py = -1;

qt =  deg2rad(30);
rad2deg(qt)
a = .05;
q0 = [0 0 0]';
lin_tol = .01;
ang_tol = .01;
iter_max = 300;
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
    error = [qt, py, px] -[qt_curr, px_curr, py_curr];
    qk = qk + a  * transpose(J_curr) * error';
    error_abs = abs(error);

    if error_abs(1) < lin_tol
        if error_abs(2) <  lin_tol
            if error_abs(3) < ang_tol
                break
            end 
        end
    end
end
figure(4);
    robot.q  = qk;
    robot_rb=defineRobot(robot,radius);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
    hold on

grid;axis([-1,3,-2,2]);axis('square');


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


