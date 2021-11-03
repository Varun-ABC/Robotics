function Phi=phi(R,p)

Phi=[R zeros(3,3);-R*hat(p) R];
