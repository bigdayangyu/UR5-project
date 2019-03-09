function gst = ur5FwdKin(q)
addpath('./basic function');

theta1 = q(1);
theta2 = q(2);
theta3 = q(3);
theta4 = q(4);
theta5 = q(5);
theta6 = q(6);

l0 = 0.0892;
l1 = 0.425;
l2 = 0.392;
l3 = 0.1093;
l4 = 0.09475;
l5 = 0.0825;


xi1 = [0;0;0;0;0;1];
xi2 = [0;l0;0;1;0;0];
xi3 = [0;l1+l0;0;1;0;0];
xi4 = [0;l1+l2+l0;0;1;0;0];
xi5 = [0;-l3;0;0;0;1];
xi6 = [0;l1+l2+l4+l0;0;1;0;0];

exp1 = twist(xi1,theta1);
exp2 = twist(xi2,theta2);
exp3 = twist(xi3,theta3);
exp4 = twist(xi4,theta4);
exp5 = twist(xi5,theta5);
exp6 = twist(xi6,theta6);

%g0 = [[0 -1 0; 1 0 0; 0 0 1], [0;l3+l5;l0+l1+l2+l4]; 0 0 0 1];
g0 = [eye(3), [l3+l5;0;l0+l1+l2+l4]; 0 0 0 1];

gst = exp1*exp2*exp3*exp4*exp5*exp6*g0;