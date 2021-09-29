clear all 
close all

if 1==2 
k = [0 0 1];
p1 = [1 0 0];
p2 = [2 2 0];
subprob0(k,p1,p2)
k'*(cross(p1,p2))

end
if 1 ==1
    
    
    L1 = 1.5;L2 = 1.5;L3 = .5;
    load S_letter_path

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
    xlabel('x-axis');ylabel('y-axis');
    title('Planar RRR arm in zero configuration (q_1=q_2=q_3=0)')
    hold off

    [xT,yT] = setR0T(out_norm);
    nl=length(Sls);
    qsol1=zeros(3,nl-1);
    qsol2=zeros(3,nl-1);
    for i=1:nl-1
        robot.T(1:3,1:4)=[xT(:,i), yT(:,i), ez, [Sls(:,i);0]];
        % tstart(i)=tic;
        qsol=threelink_invkin_geometric(robot);% <<<< you need to provide this
        % telapsed(i)=toc(tstart(i));
        qsol1(:,i)=qsol(:,1);
        qsol2(:,i)=qsol(:,2);
    end
    figure(2);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
    hold on
    grid;axis([-1,3,-2,2]);axis('square');
    for i=1:3:nl-1
        robot.q=qsol1(:,i);pause(.1);
        show(robot_rb,robot.q,'Collision','on'); 
        view(0,90);
    end
    grid;
end




function [xT,yT] = setR0T(out_norm)
    N = size(out_norm,2);
    qt= zeros(N,1);
    p1 = [1 0 0];
    k = [0 0 1];
    for i = 1:N
        qt = subprob0(k, p1, [out_norm(:,i)' 0]);
    end
    xT = [cos(qt), sin(qt), 0]';
    yT = [-sin(qt), cos(qt), 0]';
end

function Q = threelink_invkin_geometric(robot)
    % robot.T(1:3,1:4)=[xT(:,i), yT(:,i), ez, [Sls(:,i);0]];
    Target = robot.T;
    Pose_current = robot.P;
    p0 = Pose_current(1);
    xT = Target(1:3,1);
    yT = Target(1:3,2);
    cos_qt = xT(1);
    sin_qt = -yT(1);
    qt = atan2(sin_qt/ cos_qt);
    path_point = Target(1:3,4);
    p3 = path_point + [robot.L(3)*cos_qt, robot.L(3)* sin_qt, 0];
    C1 = [p3,robot.L(2)];
    C2 = [p0,robot.L(1)];
    [P2_1, P2_2] = inter_2_circ(C1, C2);
    k = [0, 0, 1];
    p1_p2_1 = P2_1 - p0;
    p1_p2_2 = P2_2 - p0;
    p2_p3_1 = p3 - P2_1;
    p2_p3_2 = p3 - P2_2;
    q1_1 = subprob0(k, [1,0,0], [p1_p2_1, 0]);
    q1_2 = subprob0(k, [1,0,0], [p1_p2_2, 0]);
    q2_1 = subprob0(l, [p1_p2_1, 0], [p2_p3_1, 0]);
    q2_2 = subprob0(l, [p1_p2_2, 0], [p2_p3_2, 0]);
    q3_1 = qt - q1_1 + q2_1;
    q3_2 = qt - q1_2 + q2_2;
    Q = [[q1_1,q2_1, q3_1], [q1_2,q2_2, q3_2]];
end
function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end

function [I1, I2] = inter_2_circ(C1, C2)
    Cent1 = C1(1);
    Cent2 = C2(1);
    r1 = C1(2);
    r2 = C2(2);
    center_dist = norm(Cent1-Cent2); % distance between circles
    cosA = (r2^2+center_dist^2-r1^2)/(2*r2*center_dist);
    u_AB = (Cent2 - Cent1)/center_dist; % unit vector from first to second center
    pu_AB = [u_AB(2), -u_AB(1)]; % perpendicular vector to unit vector
    A = Cent1 + u_AB * (r2*cosA);
    B = pu_AB * (r2*sqrt(1-cosA^2));
    I1 =  A + B;
    I2 = A - B;
    
end









function q=subprob0(k,p1,p2)

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