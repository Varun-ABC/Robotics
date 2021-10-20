%
% invkin_6DOF.m
% 
% forward kinematics using POE approach
%
% input:    robot = robot object with
%                   .T = homogeneous transformation T_{0T}
%                   .H = [h1, ..., hn] (3xn)
%                   .P = [p01, ..., p_{n-1,n},p_{n,T}] (3x(n+1))
%                   .joint_type = 1xn (0 for revolute, 1 for prismatic)
%
% output:   robot = robot object with 
%                   .q = joint displacement vector
%
% usage:
%       q = invkin_6DOF(robot);
%
% 

function robot=invkin_6DOF(robot)
    q_sol = zeros(6,8);
    ex = [1,0,0]';
    ey = [0,1,0]';
    P0t = robot.T(1:3,4);
    R0t = robot.T(1:3,1:3);
    P16 = P0t - robot.P(:,1) - R0t * robot.P(:,end);
    n_P16 = norm(P16);
    P23 = robot.P(:,3);
    P34 = robot.P(:,4);
    q3 = subprob3(robot.H(:,3), -P34, P23, n_P16);
    iter = 1;
    for i = 1:length(q3)
        q3_1 = q3(i);
        p1 = P16;
        p2 = P23 + rot(robot.H(:,3), q3_1) * P34;
        [q1,q2]=subprob2(-robot.H(:,1),robot.H(:,2),p1,p2);
        for j = 1:length(q1)
            q1_1 = q1(j);
            q2_1 = q2(j);
            k1 = -ex;
            k2 = ey;
            p1 = roty(-(q2_1 + q3_1))*rotz(-q1_1)*R0t*ex;
            p2 = ex;
            [q4,q5]=subprob2(k1,k2,p1,p2);
            for k = 1:length(q4)
                q4_1 = q4(k);
                q5_1 = q5(k);
                R03 = rot(robot.H(:,1), q1_1)*rot(robot.H(:,2), q2_1)*rot(robot.H(:,3), q3_1);
                R05 = R03 * rot(robot.H(:,4),q4_1) * rot(robot.H(:,5),q5_1);     
                q6_1 = subprob1(robot.H(:,6),robot.H(:,5),R05'*R0t*robot.H(:,5));
                q_sol(:,iter) = [q1_1, q2_1, q3_1, q4_1, q5_1, q6_1]';
                iter = iter +1;
            end
        end
    end
    robot.q = q_sol;
end







