%
% hand-eye calibration example using the soution of the general pnp problem
% 
clear all;close all;

% define a pin hole camera
cam_def

% load S-shape curve in reference frame
load S_sphere_path.mat
load S_sphere_path_uniform.mat

% define ABB IRB 1200 robot

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
L1=399.1/1000;L2=448/1000;L3=42/1000;L4=451/1000;L5=82/1000;
% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;
% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;
% 
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type = [0 0 0 0 0 0];

% **** These are the paraemters to estimate **** 
% translation of the robot base from the reference frame of the S-shape
% (assuming same orientation) represented in the reference frame
p_0B=[2.5;0;0];
% translation of the camera frame origin from end effector frame (assuming
% same orientation) represented in the end effector frame
P_TC=[0;0;0.05];
% ****

% Given an end effector location
% (specify end effector frame and then perform inverse kinematics)
irb1200.T=[[-ex -ey ez] [1.8;0;0.5]-p_0B;0 0 0 1];
% irb1200=invkinelbow(irb1200);% disp(irb1200.q);
%
% irb1200.q=[0.1;1.6;3.5;0;0;0];
%irb1200=fwddiffkiniter(irb1200);
%
T_BT=irb1200.T;

% Camera frame with respect to the end effector frame
% Note camera frame is pointing towards the S-shape as in Part 1 of
% miniproject 5
T_TC=[[ey ez ex],P_TC ;[0 0 0 1]];

% Robot location (B-frame) with respect to the reference frame
T_0B=[eye(3,3) p_0B;[0 0 0 1]];

% Generate a camera image: first calculate camera location with respect to
% the reference frame
T_0C = T_0B*T_BT*T_TC;
% generate image (same as in Part 1 of miniproject 5)
[uv,uvw,P1]=cam_image(cam,T_0C,pS);
% show camera image (same as in Part 1 of miniproject 5)
figure(30);plot(uv(1,:),uv(2,:),'x','linewidth',2);
xlabel('image plane horizontal axis');
ylabel('image plane vertical axis'); 
p0C=T_0C(1:3,4);
title(['camera frame location: ',...
    '[',num2str(p0C(1)),',',num2str(p0C(2)),',',num2str(p0C(3)),']']);
axis([0 cam.uv0(1)*2 0 cam.uv0(2)*2]);
% show image plane by flipping x axis
set (gca, 'xdir', 'reverse')

% *** This is the hand-eye calibration for Part 3
[T_C0_est,Zest]=pnp_general(uv,pS,cam.K);
T_0C_est=inv(T_C0_est);

% check for error 
%%
T_0C
T_0C_est
disp(['Error in Pose ', num2str(norm(T_0C-T_0C_est))]);
