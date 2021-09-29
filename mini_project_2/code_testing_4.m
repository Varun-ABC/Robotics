if 1 ==2
load S_letter_path

figure(1);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
xlabel('x');ylabel('y');
hold on
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

L1 = 1.5;
L2 = 1.5;
L3 = .5;
P0=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
robot.P = [P0, L1*ex, L2*ex, L3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];
robot.q=[0;0;0];

py = 1;
px = 2.5375;
% qt = deg2rad(30);
qt = deg2rad(-45);

L = [1.5,1.5,.5];

robot.L = L;

P3x = px - L(3) * cos(qt);
P3y = py - L(3) * sin(qt);
delta = abs(P3x^2) + abs(P3y^2);
cos_q2 = (delta - abs(L(1)^2) - abs(L(2)^2))/ (2 * L(1)*L(2));
sin_q2 = (1-abs(cos_q2^2))^.5;
q2 = atan2(sin_q2, cos_q2);
sin_q1 = ((L(1)+ L(2)*cos_q2) * P3y - L(2)*sin_q2 * P3x)/ delta;
cos_q1 = ((L(1)+ L(2)*cos_q2) * P3x + L(2)*sin_q2 * P3y)/ delta;
q1 = atan2(sin_q1, cos_q1);
q3 = qt - (q1 + q2);
Q = [q1, q2, q3]';
robot.q = Q;
radius = .01;
robot_rb=defineRobot(robot,radius);
show(robot_rb,robot.q,'Collision','on'); 
view(0,90);
end



k = [0 0 1];
p1 = [1 0 0];
p2 = [1 1 0];
rad2deg(subprob0_1(k,p1,p2))



function [T, J] = forward_kin(robot)
    Q = robot.q;
    L = robot.L;
    qt= sum(Q);
    Q_cum = cumsum(Q);
    R0t = rot2(qt);
    Yt = L(1) * sin(Q_cum(1)) + L(2) * sin(Q_cum(2)) + L(3) * sin(Q_cum(3));
    Xt = L(1) * cos(Q_cum(1)) + L(2) * cos(Q_cum(2)) + L(3) * cos(Q_cum(3));
    T = eye(4);
    T(1:2,1:2) = R0t;
    T(1,end) = Xt; 
    T(2,end) = Yt; 
    J = eye(3);
end
% T; end effector pose T
function Q = threelink_invkin_geometric(robot)
    % robot.T(1:3,1:4)=[xT(:,i), yT(:,i), ez, [Sls(:,i);0]];
    Target = robot.T;
    Pose_current = robot.P;
    p0 = Pose_current(1:3, 1);
    xT = Target(1:3,1);
    yT = Target(1:3,2);
    cos_qt = xT(1);
    sin_qt = -yT(1);
    qt = atan2(sin_qt, cos_qt);
    path_point = Target(1:3,4);
    path_point
    p3 = path_point + [robot.L(3)*cos_qt, robot.L(3)* sin_qt, 0]';
    C1 = p3(1:2);
    C2 = p0(1:2);
    [P2_1, P2_2] = inter_2_circ(C1, robot.L(2), C2, robot.L(1));
    k = [0, 0, 1];
    C1
    robot.L(2)
    C2
    robot.L(1)
    P2_1 = [P2_1';0]
    P2_2 = [P2_2'; 0];

    p0
    p1_p2_1 = P2_1 - p0;
    p1_p2_1
    p1_p2_2 = P2_2 - p0;
    p2_p3_1 = p3 - P2_1;
    p2_p3_2 = p3 - P2_2;
    q1_1 = subprob0_1(k, [1,0,0], [p1_p2_1', 0]);
    q1_2 = subprob0_1(k, [1,0,0], [p1_p2_2', 0]);
    q2_1 = subprob0_1(l, [p1_p2_1, 0], [p2_p3_1', 0]);
    q2_2 = subprob0_1(l, [p1_p2_2, 0], [p2_p3_2', 0]);
    q3_1 = qt - q1_1 + q2_1;
    q3_2 = qt - q1_2 + q2_2;
    Q = [[q1_1,q2_1, q3_1], [q1_2,q2_2, q3_2]];
end


function Q = threelink_invkin_algebraic(robot)
    Target = robot.T;
    path_point = Target(1:3,4)
    px = path_point(1);
    py = path_point(2);
    Pose_current = robot.P;
    p0 = Pose_current(1:3, 1);
    xT = Target(1:3,1);
    cos_qt = xT(1);
    sin_qt = xT(2);
    qt = atan2(sin_qt, cos_qt);
    L = robot.L;
    P3x = px - L(3) * cos(qt);
    P3y = py - L(3) * sin(qt);
    delta = abs(P3x^2) + abs(P3y^2);
    cos_q2 = (delta - abs(L(1)^2) - abs(L(2)^2))/ (2 * L(1)*L(2));
    sin_q2 = (1-abs(cos_q2^2))^.5;
    q2 = atan2(sin_q2, cos_q2);
    sin_q1 = ((L(1)+ abs(L(2)^2)) * P3y - L(2)*sin_q2 * P3x)/ delta;
    cos_q1 = ((L(1) + abs(L(2)^2)) * P3x - L(2)*sin_q2 * P3y)/ delta;
    q1 = atan2(sin_q1, cos_q1);
    q3 = qt - (q1 + q2);
    Q = [q1, q2, q3]
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
    I1
    I2 = A - B;
    
end

function [xT,yT] = setR0T(out_norm)
    N = size(out_norm,2);
    qt= zeros(N,1);
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
