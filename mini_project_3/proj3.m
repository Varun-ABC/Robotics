%
%
% 
clear all; close all;
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

load S_sphere_path

% plot the spherical S
figure(1);plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
title("3D S-curve")
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
hold on;
% add a 3d sphere
surf(X,Y,Z)
% make it transparent
alpha .5
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% convert to equal path length grid
diffS=vecnorm(diff(p_S')');
ls=[0 cumsum(diffS)];
lf=sum(diffS);
N=100;
l=(0:lf/N:lf);

pS=interp1(ls,p_S',l,'spline')';
% plot it out again with equal path length
figure(2);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
title("3D S-curve with Outward Normal")
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% check the path length is indeed equal
dlvec=vecnorm(diff(pS')');
figure(3);plot(dlvec,'x')
title("Difference in Path Length from mean")
xlabel("Point Number")
ylabel("Reletive Error (m)")
dl = mean(dlvec);
disp(max(abs(dlvec-dl)));

% save it as the path file
save S_sphere_path_uniform l pS

% find the end effector frame
pc=r*ez;
N=length(pS);
xT=zeros(3,N);zT=zeros(3,N);yT=zeros(3,N);
figure(2)
for i=1:N
    xT(:,i)=(pS(:,i)-pc)/norm(pS(:,i)-pc);
    if i<N
        yT(:,i)=(pS(:,i+1)-pS(:,i));
    else
        yT(:,i)=yT(:,i-1);
    end
    yT(:,i)=yT(:,i)-yT(:,i)'*xT(:,i)*xT(:,i);
    yT(:,i)=yT(:,i)/norm(yT(:,i));
    zT(:,i)=cross(xT(:,i),yT(:,i));
    R = [xT(:,i) yT(:,i) zT(:,i)];
    
    quiver3(pS(1,i),pS(2,i),pS(3,i), xT(1,i),xT(2,i),xT(3,i))
    hold on
end

% plot out the end effector frame
% m=5;
% MATLAB's plotTransforms command plots a frame at a given location
% figure(2);h=plotTransforms(pS(:,1:5:end)',quat(:,1:5:end)', 'FrameSize', );
% set(h,'LineWidth',.5);

% ABB IRB 1200 parameters

L1=399.1;
L2=448;
L3=42;
L4=451;
L5=82;

%POE Parameter

% P
p01=0*ex+L1*ez;
p12=zz;
p23=L2*ez;
p34=L3*ez+L4*ex;
p45=zz;
p56=zz;
p6T=L5*ex;

% H
h1=ez;
h2=ey;
h3=ey;
h4=ex;
h5=ey;
h6=ex;

% Final transformation

% define abb 1200 robot using POE convention
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T]/1000;
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];
irb1200.L = [L1,L2,L3,L4,L5];
%irb1200.R6T=R6T;

% define collision body for abb 1200
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);
 
% 
% find all inverse kinematics solutions
%

for i=1:N
    % specify end effector SE(3) frame
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)] pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    %
    irb1200 = invkin_6DOF(irb1200); % << you need to supply this!!!
    %
    for k=1:8
        q(:,i,k) = irb1200.q(:,k);
    end
        % check forward kinematics to make sure the IK solution is correct
    for k=1:8
        irb1200.q = q(:,i,k);
        irb1200 = fwdkiniter(irb1200);
        T_MDH{i,k} = MDH_Check(irb1200);
        T_SDH{i,k} = SDH_Check(irb1200);
        T_check{i,k}=irb1200.T;
    end
end
%% Sphere with outward Normal
% choose the pose to visualize
writerObj = VideoWriter("SixDOFArm.mp4");
open(writerObj);

% for k = 1:8
%     figure(20+k);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
%     title(["Arm Motion for Pose:" num2str(k)]);
%     xlabel('x');ylabel('y');zlabel('z');
%     hold on;
%     % 3d sphere
%     surf(X,Y,Z)
%     % make it transparent
%     alpha 0.4
%     axis(r*[-1 1 -1 1 0 2]);axis('square');
%     view(120,10);
%     hold on
% end

figure(4);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
title("Sphere with Outward Normal")
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
hold on;
quiver3(pS(1,i),pS(2,i),pS(3,i), xT(1,i),xT(2,i),xT(3,i))
hold on
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);
mov_iter = 0;
%% Rotation Matrix Represetations
quat=zeros(4,N); % allocate space for unit quaternion representation of R_{0T}
Ang_Axis=zeros(4,N);
Euler=zeros(3,N);

for i=1:N
    xT(:,i)=(pS(:,i)-pc)/norm(pS(:,i)-pc);
    if i<N
        yT(:,i)=(pS(:,i+1)-pS(:,i));
    else
        yT(:,i)=yT(:,i-1);
    end
    yT(:,i)=yT(:,i)-yT(:,i)'*xT(:,i)*xT(:,i);
    yT(:,i)=yT(:,i)/norm(yT(:,i));
    zT(:,i)=cross(xT(:,i),yT(:,i));
    R = [xT(:,i) yT(:,i) zT(:,i)];
    quat(:,i)=R2q(R);
    Euler(:,i) = R2Euler(R)';
    Ang_Axis(:,i) = R2AxisAngle(R)';
end
figure(100)
title("Quaternion Angle Representation as a function of Lambda")
legend("Rational Part","i","j","k")
xlabel("Lambda (m)")
plot(l,quat);
figure(200)
title("Euler Angle Representation as a function of Lambda")
legend("Roll","Pitch","Yaw")
xlabel("Lambda (m)")

plot(l,Euler)
figure(300)
plot(l,Ang_Axis)
title("Angle Axis Representation as a function of Lambda")
legend("Angle","X","Y","Z")
xlabel("Lambda (m)")

%% plotting arm motion
figure(2)
for ksol = 6
    for i=1:N
        % show robot pose (ever 5 frames)
        if mod(i,5)==0
            mov_iter = mov_iter+1;
            show(irb1200_rbt,q(:,i,ksol),'collision','on');
            M(mov_iter) = getframe;
            view(150,10);
            writeVideo(writerObj , M(mov_iter))
        end
    end
end
close(writerObj);
% movie(M)

%% error
error_SDH = zeros(8, N);
error_MDH = zeros(8, N);
error = zeros(8, N);
for ksol = 1:8
    for i=1:N
        error(ksol, i) = norm(T_check{i,ksol}-Td{i});
        error_MDH(ksol, i) = norm(Td{i}- T_MDH{i,ksol});
        error_SDH(ksol, i) = norm(Td{i}- T_SDH{i,ksol});
    end
end
figure(66)
title("Error Plot for all 8 Poses");xlabel("Point on Path");ylabel("Reletive Error (mm)");
for i = 1:8
    scatter(1:N, error(i,:));hold on
end
figure(67)
for i = 1:8
    title("MDH Error Plot for all 8 Poses");xlabel("Point on Path");ylabel("Reletive Error (mm)");
    scatter(1:N, error_MDH(i,:));hold on

end
figure(68)
for i = 1:8
    title("SDH Error Plot for all 8 Poses");xlabel("Point on Path");ylabel("Reletive Error (mm)");
    scatter(1:N, error_SDH(i,:));hold on
end

%%
% need to run Section 1 first!!
% compute end effector linear and angular velcoity 

path_vel = zeros(N-1,8);
% find the largest angle traveled by the robot
dt = zeros(8,1);
max_qdot = 2;
dq = zeros(6,100,8);
for k = 1:8
     for i = 1:length(q(:,:,k))-1
         dq(:,i,k)= (wrapToPi(q(:,i+1,k))-wrapToPi(q(:,i,k)));
     end 
    max_dq_ar = max(abs(dq(:,:,k)));
    max_dq = max(max_dq_ar);
    dt(k) = abs(max_dq)/max_qdot;
end
for k=1:8
    for i=1:N-1
 %  dt(i) = (ls(i+1)-ls(i))/lsdot;
        qdot(:,i,k)= dq(:,i,k)/dt(k);
        Ri1=T_check{i+1,k}(1:3,1:3);
        Ri=T_check{i,k}(1:3,1:3);
        w(:,i,k)=vee(Ri1*Ri'-eye(3,3))/dt(k);
        pi1=T_check{i+1,k}(1:3,4);
        p_i = T_check{i,k}(1:3,4);
        v(:,i,k)=(pi1-p_i)/dt(k);
        path_vel(i,k) = norm(v(:,i,k));
    end
end
spatial_velocity  = [v;w];
max_path = zeros(8,2);
q3_var = zeros(8,1);
figure(800)
for k = 1:8
    plot(l(1:end-1), qdot(3,:,k))
    hold on
end
legend("Pose 1","Pose 2","Pose 3","Pose 4","Pose 5", "Pose 6", "Pose 7","Pose 8")
xlabel("Lambda (m)")
ylabel("Qdot (rad/sec)")
for k=1:8
   for i=1:6
       maxqdot(i,k)=max(qdot(i,:,k));
       figure(30*k)
       plot(l(1:end-1), qdot(i,:,k))
       hold on
   end
   legend("q1","q2",'q3','q4','q5','q6')
   title("qdot for pose %d \n", k)
   xlabel("Lambda (m)")
   ylabel("qdot (rad/sec)")
   hold off
   fprintf('maximum qdot for pose %d \n', k);
   disp(maxqdot(:,k)');
   q3_var(k) = std(qdot(3,:,k));
   
end
figure(40*k)
plot(l(1:end-1), path_vel(:,1));hold on
plot(l(1:end-1), path_vel(:,2));hold on
plot(l(1:end-1), path_vel(:,3));hold on
plot(l(1:end-1), path_vel(:,4));hold on
plot(l(1:end-1), path_vel(:,5));hold on
plot(l(1:end-1), path_vel(:,6));hold on
plot(l(1:end-1), path_vel(:,7));hold on
plot(l(1:end-1), path_vel(:,8));hold on
legend("Path 1","Path 2","Path 3",'Path 4','Path 5','Path 6','Path 7','Path 8')
title("Path Velocity as a function of Lambda")
xlabel("Lambda (m)")
ylabel("Path Velocity (m/s)")
hold off
[max_path_v, path_num] = max(max(path_vel));
fprintf('Maximum Path Velocity in pose %d \n',path_num);
fprintf('Maximum Path Velocity (m/s) %d \n',max_path_v);

[q3_var_val, q3_var_ind]= min(q3_var);
fprintf('Minimum variation for q3 in pose %d \n',q3_var_ind);
fprintf('Minimum standard devation for q3 (rad) %d \n',q3_var_val);


%%
%
% R2q.m
%
% converts R in SO(3) to unit quaternion q, (q0,q_vec)
%

function q=R2q(R)
  
  q=zeros(4,1);
  q(1)=.5*sqrt(trace(R)+1);
  if abs(q(1))<1e-5
    [k,q]=R2kth(R);
    q(2:4)=k;
  else
    q(2:4)= vee(R-R')/4/q(1);
  end
end

function diff_SDH_POE = SDH_Check(robot)
    q1 = robot.q(1);
    q2 = robot.q(2);
    q3 = robot.q(3);
    q4 = robot.q(4);
    q5 = robot.q(5);
    q6 = robot.q(6);
    
    l1 = robot.L(1);
    l2 = robot.L(2);
    l3 = robot.L(3);
    l4 = robot.L(4);
    l5 = robot.L(5);
    
    d=[l1; 0; 0; l4; 0; l5];
    a=[0; l2; l3; 0; 0; 0];
    alpha=[-pi/2; 0; -pi/2; pi/2; -pi/2; 0];
    theta=[q1; q2-pi/2; q3; q4; q5; q6+pi/2];
    Tr = eye(4);
    for i = 1:length(a)
        T_SDH(:,:,i)=[cos(theta(i)), -cos(alpha(i))*sin(theta(i)), sin(alpha(i))*sin(theta(i)), a(i)*cos(theta(i));...
            sin(theta(i)), cos(alpha(i))*sin(theta(i)), -sin(alpha(i))*cos(theta(i)), a(i)*sin(theta(i));...
            0, sin(alpha(i)), cos(alpha(i)), d(i);...
            0, 0, 0, 1];
    end
    for i= 1:length(a)
        Tr=Tr*T_SDH(:,:,i);
    end
    
    diff_SDH_POE = Tr;
end
    


function diff_MDH_POE = MDH_Check(robot)
    q1 = robot.q(1);
    q2 = robot.q(2);
    q3 = robot.q(3);
    q4 = robot.q(4);
    q5 = robot.q(5);
    q6 = robot.q(6);
    
    l1 = robot.L(1);
    l2 = robot.L(2);
    l3 = robot.L(3);
    l4 = robot.L(4);
    l5 = robot.L(5);
    
    d=[l1;0;0;l4;0;0;l5];
    a=[0;0;-l3;0;0;0;0]; % this is a_{i-1}
    alpha=[0; -pi/2; 0; pi/2; -pi/2; pi/2; 0];
    theta=[q1;q2+pi/2;q3;q4;q5;q6;0];
    Tr = eye(4);

    for i = 1:length(a)
        T_MDH(:,:,i)=[cos(theta(i)), sin(theta(i)), 0,0;...
            cos(alpha(i))*sin(theta(i)), cos(alpha(i))*cos(theta(i)), -sin(alpha(i)), -d(i)*sin(alpha(i));...
            sin(alpha(i))*sin(theta(i)), sin(alpha(i))*cos(theta(i)), cos(alpha(i)), d(i)*cos(alpha(i));...
            0, 0, 0, 1];
    end
    for i= 1:length(a)
        Tr=Tr*T_MDH(:,:,i);
    end
    
    diff_MDH_POE = Tr;
end

function  Eul = R2Euler(R)
        beta2 = asin(R(3, 1));
        beta3 = atan2(-R(3, 2) / cos(beta2),R(3, 3) / cos(beta2));
        beta1 = atan2(-R(2, 1) / cos(beta2), R(1, 1) / cos(beta2)); 
        Eul = [beta1, beta2, beta3];
end

function A = R2AxisAngle(R)

    cos_angle = (trace(R) - 1.0) / 2.0;
    Angle = acos(min(max(-1.0, cos_angle), 1.0));
    axis = [R(3, 2) - R(2, 3), R(1, 3) - R(3, 1), R(2, 1) - R(1, 2)];
    Axis = axis/norm(axis);
    A = [Angle, Axis];
end