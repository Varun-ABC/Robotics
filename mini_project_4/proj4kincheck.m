%
% proj4kincheck.m
%
% forward/inverse fkinematics checker for an elbow arm
% using MATLAB inversekinematics solver vs. subproblem decomposition
% 
% make sure fwdkin_example_rbt is in the path to generate the elbow_rbt
% rigid body tree
% 
% output plots compare EE solution accuracy and computation times
%

%clear all; close all;

% MATLAB robotics toolbox inverse kinematics object

ik = inverseKinematics('RigidBodyTree',irb1200_rbt);

% set storage
N_check = 50;

n=length(irb1200.H);
q=zeros(n,N_check);
qsol=zeros(n,N_check);
qsol1=zeros(n,8,N_check);
qsol2=zeros(n,N_check);
T=zeros(4,4,N_check);
Tsol=zeros(4,4,N_check);
Tsol1=zeros(4,4,8,N_check);
Jsol2=zeros(6,6,N_check);
errT=zeros(N_check,1);
errT1=zeros(N_check,8);
errT2=zeros(N_check,1);
telapsed=zeros(1,N_check);
telapsed1=zeros(1,N_check);
telapsed2=zeros(1,N_check);

for i=1:N_check
    % random arm configuration
    q(:,i)=(rand(6,1)-.5)*pi;
    irb1200.q = q(:,i);
    
    % forward kinematics
    irb1200=fwdkiniter(irb1200);
    % nominal end effector pose
    T(:,:,i)=irb1200.T;
    % MATLAB's inverse kinematics
    % keep track of the computation time
    tstart=tic;
    [qsol(:,i),solnInfo]=...
        ik('body7',irb1200.T,ones(1,6),q(:,i) + q(:,i)*.1*randn);
    telapsed(i) = toc(tstart);    
    % forward kinematics again to compare with EE pose
    irb1200.q=qsol(:,i);
    irb1200=fwdkiniter(irb1200);
    Tsol(:,:,i)=irb1200.T;
    % now use our own exact inverse kinematics
    irb1200.q = q(:,i);
    irb1200.T = T(:,:,i);
    % keep track of the computation time
    tstart=tic;
    irb1200=invkinelbow(irb1200);
    telapsed1(i) = toc(tstart);
    % there are 8 solutions
    qsol1(:,:,i)=irb1200.q;
    % find all the EE pose
    for j=1:8
        irb1200.q=qsol1(:,j,i);
        irb1200=fwddiffkiniter(irb1200);
        Tsol1(:,:,j,i)=irb1200.T;
        Jsol1(:,:,j,i)=irb1200.J;
        errT1(i,j)=norm(T(:,:,i)-Tsol1(:,:,j,i),'fro');
    end
    % find EE pose error
    errT(i)=norm(T(:,:,i)-Tsol(:,:,i),'fro');
    % our own iterative inverse kinematics

    irb1200.Weights=[1.5;1.5;1.5;1;1;1];
    irb1200.q = q(:,i) + q(:,i).*.1.*randn(6,1);
    % irb1200.q = (rand(6,1)-.5)*pi;
    irb1200.T = T(:,:,i);
    tstart=tic;
    irb1200=invkin_iterJ(irb1200, 300, .2, .005);
    mm(i) = min(svd(irb1200.J));
    telapsed2(i) = toc(tstart);
    qsol2(:,i)=irb1200.q;
    Tsol2(:,:,i)=irb1200.T;
    Jsol2(:,:,i)=irb1200.J;
    errT2(i)=norm(T(:,:,i) - Tsol2(:,:,i),'fro');
end 

figure(33);plot((1:N_check),errT,'bx',(1:N_check),errT2,'linewidth',2); hold on;
for j=1:8
    plot((1:N_check),errT1(:,j),'ro','linewidth',2);
end
hold off;
set(gca, 'YScale', 'log')
ylim([0,4e-4])
xlabel('random test number');ylabel('end effector error');
title('End effector pose error || T - T_{solve} ||_F');
legend('MATLAB','iterative Jacobian','exact by subproblem');

fprintf('max MATLAB EE error: %g \n',max(errT));
fprintf('max subproblem EE error: %g \n',max(max(errT1)));
fprintf('max iterative Jacobian EE error: %g \n',max(max(errT2)));

figure(20);plot((1:N_check),telapsed,'bx',(1:N_check),telapsed1,'ro',...,
    (1:N_check),telapsed2,'g^','linewidth',2);
xlabel('random test number');ylabel('elapsed time (sec)');
title('inverse kinematics computation time');
legend('MATLAB','exact by subproblem','iterative Jacobian');


fprintf('max and average MATLAB invkin computation time: %g, %g \n',...
    max(telapsed),mean(telapsed));
fprintf('max and average subproblem invkin computation time: %g, %g \n',...
    max(telapsed1),mean(telapsed1));
fprintf('max and average iterative invkin computation time: %g, %g \n',...
    max(telapsed2),mean(telapsed2));


