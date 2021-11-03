%
% fwdkiniter.m
% 
% forward kinematics using POE approach
%
% input:    robot = robot object with
%                   .q = joint displacement vector
%                   .H = [h1, ..., hn] (3xn)
%                   .P = [p01, ..., p_{n-1,n},p_{n,T}] (3x(n+1))
%                   .joint_type = 1xn (0 for revolute, 1 for prismatic)
%
% output:   robot = robot object with 
%                   .T = homogeneous transformation T_{0T}
%                   .J = Jacobian J_T in the 0 frame
%
% usage:
%       robot = fwddiffkiniter(robot);   
%
% 

function robot=fwdkiniter(robot)

q=robot.q;
n=length(robot.q);
P = robot.P;
T=eye(4,4);
J = zeros(6,n);
joint_type = robot.joint_type;
if ~exist('joint_type')
    robot.joint_type=zeros(1,n);
end

for i=1:n
    h=robot.H(1:3,i);
    if robot.joint_type(i)==0
        R = expm(hat(h)*q(i));
        p = robot.P(1:3,i);
    else
        R=eye(3,3);
        p=robot.P(1:3,i)+q(i)*h;
    end
    J = phi(eye(3,3), T(1:3, 1:3)*p)*J;
    if robot.joint_type(i)==0
        J(:,i) = [T(1:3, 1:3) * h; zeros(3,1)];
    
    elseif robot.joint_type(i)==1
        J(:,i) = [zeros(3,1); T(1:3, 1:3) * h; zeros(3,1)];
    end
    T=T*[R p;zeros(1,3) 1];
end    
robot.J = phi(eye(3,3), T(1:3, 1:3)*P(:,n+1))*J;
robot.T= T*[eye(3,3) robot.P(1:3,n+1);zeros(1,3) 1];
end

% cross production function
function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  
end

