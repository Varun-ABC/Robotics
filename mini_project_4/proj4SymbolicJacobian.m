% mini-project 4 example script
% 
%clear all; close all;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% symbolic ABB IRB 1200 robot

syms L1 L2 L3 L4 L5 positive
syms q1 q2 q3 q4 q5 q6 real

% define ABB IRB 1200 robot symbolically

% P
p01=0*ex+L1*ez;
p12=zz;
p23=L2*ez;
p34=L3*ez+L4*ex;
p45=zz;
p56=zz;
p6T=L5*ex;
%p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L4*ex;p45=zz;p56=zz;p6T=L5*ex;

% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;

% q
q=[q1;q2;q3;q4;q5;q6];

% forward kinematics and Jacobian

irb1200_s.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200_s.H=[h1 h2 h3 h4 h5 h6];
irb1200_s.joint_type=[0 0 0 0 0 0];
irb1200_s.q=q;

irb1200_s=fwdkiniter(irb1200_s);


JT_0=irb1200_s.J;
R0T=simplify(irb1200_s.T(1:3,1:3));
R03 = rot(h1,q1)*rot(h2,q2) * rot(h3,q3);

% analytical Jacobian in base frame

J4_3_given = simplify(phi(R03',-R0T*p6T)*JT_0);
pretty(J4_3_given)

R31 = (rot(h2,q2) * rot(h3,q3))';
R32 = rot(h3,q3)';
R33 = eye(3);
R34 = rot(h4,q4);
R35 = rot(h4,q4)* rot(h5,q5);
R36 = rot(h4,q4)* rot(h5,q5) * rot(h6,q6); 

h1_3 = R31 * h1;
h2_3 = R32 * h2;

h3_3 = R33 * h3; h4_3 = R34 * h4;
h5_3 = R35 * h5; h6_3 = R36 *  h6;

p14_3 = R31 * p12 + R32 * p23 + R33 * p34;
p24_3 = R32 * p23 + R33 * p34;
p34_3 = R33 * p34;
p44_3 = [0,0,0]'; p54_3 = [0,0,0]'; p64_3 = [0,0,0]';

J4_3(:,1) = [h1_3; hat(h1_3) * p14_3];
J4_3(:,2) = [h2_3; hat(h2_3) * p24_3];
J4_3(:,3) = [h3_3; hat(h3_3) * p34_3];
J4_3(:,4) = [h4_3; hat(h4_3) * p44_3];
J4_3(:,5) = [h5_3; hat(h5_3) * p54_3];
J4_3(:,6) = [h6_3; hat(h6_3) * p64_3];

J4_3 = simplify(J4_3);
pretty(J4_3)
simplify(J4_3 - J4_3_given)

% save IRB1200Jacobian JT_0 J4_3

%
% phi.m
% 
% propagation of spatial velocity
%
function phimat=phi(R,p)
          
    phimat=[R zeros(3,3);-R*hat(p) R];
    
end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end
