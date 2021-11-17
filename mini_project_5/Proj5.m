clear all ; close all;
% Initilaization Code
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

addpath(".\Robot_functions")
addpath(".\given_code")
addpath("..\mini_project_3")
addpath("..\MATAB utility code")
load S_sphere_path_uniform
cam_def
%% Part 1
Xc_1 = 1.2;
Poc_1 = [Xc_1; 0; 0.5];
Toc_1 = eye(4);
Toc_1(1:3,1) = -ey;
Toc_1(1:3,2) = ez;
Toc_1(1:3,3) = -ex;
Toc_1(1:3,4) = Poc_1;
[uv_1,uvw_1,P1_1]=cam_image(cam,Toc_1,pS); 
figure(1)
plot(uv_1(1,:),uv_1(2,:),'x', 'linewidth',3);
xlim([0, 1300])
% ylim([0,1300])
axis equal
title("Pinhole Camera to S, Close View")
xlabel("X-axis (pixels)")
ylabel("Y-axis (pixels)")
set (gca, 'xdir', 'reverse')

Xc_2 = 2.5;
Poc_2 = [Xc_2; 0; 0.5];
Toc_2 = eye(4);
Toc_2(1:3,1) = -ey;
Toc_2(1:3,2) = ez;
Toc_2(1:3,3) = -ex;
Toc_2(1:3,4) = Poc_2;
[uv_2,uvw_2,P1_2]=cam_image(cam,Toc_2,pS); 
figure(2)
plot(uv_2(1,:),uv_2(2,:),'x', 'linewidth',3);
xlim([0,1300])
ylim([0,1100])
title("Pinhole Camera to S, Far View")
xlabel("X-axis (pixels)")
ylabel("Y-axis (pixels)")
set (gca, 'xdir', 'reverse')
axis equal
%% Part 2

% camcalib.m: camera calibration script (it calls the following routines)
% cam def.m: pin hold camera object definition
% target def.m: a pattern of planar dots as the calibration target
% homographymat.m: solution of the Homography matrix
% cam image.m: generation of of a camera image from a pattern of 3D points.

camcalib

%% Part 3
% To find the extrensic parameters run the following code

hand_eye_cal

%% Part 4a: PBVS


proj5VS

%% Part 4b: IBVS


proj5IBVS

