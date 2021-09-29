A = [0 0]'; %# center of the first circle
B = [1 0]'; %# center of the second circle
a = 0.7; %# radius of the SECOND circle
b = 0.9; %# radius of the FIRST circle
c = norm(A-B); %# distance between circles

cosAlpha = (b^2+c^2-a^2)/(2*b*c);

u_AB = (B - A)/c; %# unit vector from first to second center
pu_AB = [u_AB(2), -u_AB(1)]; %# perpendicular vector to unit vector

%# use the cosine of alpha to calculate the length of the
%# vector along and perpendicular to AB that leads to the
%# intersection point
norm_vec = pu_AB * (b*sqrt(1-cosAlpha^2));
intersect_1 = A + u_AB * (b*cosAlpha) + norm_vec;
intersect_2 = A + u_AB * (b*cosAlpha) - norm_vec;
