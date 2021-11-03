% mini-project 4 example script
% 
load S_sphere_path

% initialize for the project including IRB1200 structure and rigid body
% tree
addpath(".\Robot_functions")
addpath(".\Given Code")
addpath("..\mini_project_3")

proj4init;

% symbolic Jacobian 

proj4SymbolicJacobian

% Jacobian computation comparison between three methods

proj4JacobianComparison

% Jacobian based inverse kinematics comparison

proj4kincheck
%% INVERSE ITEATIVE KINEMATICS 
irb1200.q = [0,0,0,0,0,0];
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);
N = length(yT);
q = ones(6,N);
for i=1:N
    % specify end effector SE(3) frame
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)] pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    %
    irb1200.Weights=[.5;.5;.5;1;1;1];
    irb1200=invkin_iterJ(irb1200, 500, .2, .1);
    q(:,i) = irb1200.q;
end

figure(2)
for i=1:N
    
    quiver3(pS(1,i),pS(2,i),pS(3,i), xT(1,i),xT(2,i),xT(3,i))
    hold on
end

plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
title("3D S-curve with Outward Normal")
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
hold on;
% 3d sphere
load S_sphere_path
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);
writerObj = VideoWriter("SixDOFArm_iter.mp4");
open(writerObj);
figure(2)
mov_iter = 0;
for i=1:N
    % show robot pose (ever 5 frames)
    if mod(i,5)==0
        mov_iter = mov_iter+1;
        show(irb1200_rbt, q(:,i) ,'collision','on');
        hold on
        pause(2)
        M(mov_iter) = getframe;
        view(150,10);
        writeVideo(writerObj , M(mov_iter))
    end
end
hold off
close(writerObj);

%% Jacobian and Trajectory Motion
N = length(xT);
for i=1:N
    % specify end effector SE(3) frame
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)] pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    %
    irb1200 = invkin_6DOF(irb1200); % << you need to supply this!!!
    %
    for k=1:8
        q_sub(:,i,k) = irb1200.q(:,k);
    end
end
min_sing_val = zeros(1,length(q_sub(:,:,8)));
for k = 1:8
    for i = 1:length(q_sub(:,:,8))
        
        %     irb1200.q = q(:,i,8);
        %     double(irb1200.q)
        %     irb1200 = fwdkiniter(irb1200);
        q_loop = q_sub(:,i,k);
        J4_3_num = J4_3func(q_loop);
        R03=rot(irb1200.H(:,1),q_loop(1))*rot(irb1200.H(:,2),q_loop(2))*rot(irb1200.H(:,3),q_loop(3));
        R3T=rot(irb1200.H(:,4),q_loop(4))*rot(irb1200.H(:,5),q_loop(5))*rot(irb1200.H(:,6),q_loop(6));
        JT_0=phi(R03,R3T*irb1200.P(:,end))*J4_3_num;
        min_sing_val(i) = min(svd(JT_0));
    end
     figure(10*k)
     plot(l, min_sing_val)
     title(["Minimum Singular Value for Pose " num2str(k)])
     ylabel('Minimum Singular Value')
     xlabel('lambda (m)')
end

%% Show arm in singular positions
% boundary condition 
q_show = [0,0,atan(-L4/ L3),0,0,0]';
figure(111)
show(irb1200_rbt, q_show ,'collision','on');
q_show = [0,pi/2,0,0,0,0]';
show(irb1200_rbt, q_show ,'collision','on');
q2 = (-L4 * cos(0) - L3 * sin(0))/(-L4 * sin(0) + L3 * cos(0) + L2);
q_show = [0,q2,0,0,0,0]';
show(irb1200_rbt, q_show ,'collision','on');

%%
function J=J4_3func(q)
L2=.448;
L3=.042;
L4=.451;

J=[-sin(q(2) + q(3)),   0,  0,  1,  0,  cos(q(5));
    0,   1,     1,   0,      cos(q(4)),  sin(q(4))*sin(q(5));
    cos(q(2) + q(3)), 0,   0, 0, sin(q(4)), -cos(q(4))*sin(q(5));
    0, L3 + L2*cos(q(3)),  L3, 0,       0, 0;
    L4*cos(q(2) + q(3)) + L3*sin(q(2) + q(3)) + L2*sin(q(2)), 0, 0, 0,  0, 0;
    0, L2*sin(q(3)) - L4, -L4, 0, 0, 0]; 

end

