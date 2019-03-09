function Jb = ur5BodyJacobian(q)
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

g1 = inv(exp1*exp2*exp3*exp4*exp5*exp6*g0);
g2 = inv(exp2*exp3*exp4*exp5*exp6*g0);
g3 = inv(exp3*exp4*exp5*exp6*g0);
g4 = inv(exp4*exp5*exp6*g0);
g5 = inv(exp5*exp6*g0);
g6 = inv(exp6*g0);

R1 = g1(1:3,1:3);
R2 = g2(1:3,1:3);
R3 = g3(1:3,1:3);
R4 = g4(1:3,1:3);
R5 = g5(1:3,1:3);
R6 = g6(1:3,1:3);

p1 = g1(1:3,4);
p2 = g2(1:3,4);
p3 = g3(1:3,4);
p4 = g4(1:3,4);
p5 = g5(1:3,4);
p6 = g6(1:3,4);

p1_hat = SKEW3(p1);
p2_hat = SKEW3(p2);
p3_hat = SKEW3(p3);
p4_hat = SKEW3(p4);
p5_hat = SKEW3(p5);
p6_hat = SKEW3(p6);

Ad1 = [R1, p1_hat * R1; zeros(3,3), R1];
Ad2 = [R2, p2_hat * R2; zeros(3,3), R2];
Ad3 = [R3, p3_hat * R3; zeros(3,3), R3];
Ad4 = [R4, p4_hat * R4; zeros(3,3), R4];
Ad5 = [R5, p5_hat * R5; zeros(3,3), R5];
Ad6 = [R6, p6_hat * R6; zeros(3,3), R6];

xi1_prim = Ad1 * xi1;
xi2_prim = Ad2 * xi2;
xi3_prim = Ad3 * xi3;
xi4_prim = Ad4 * xi4;
xi5_prim = Ad5 * xi5;
xi6_prim = Ad6 * xi6;

Jb = [xi1_prim,xi2_prim,xi3_prim,xi4_prim,xi5_prim,xi6_prim];
