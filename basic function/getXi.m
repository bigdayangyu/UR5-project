function xi = getXi(g)
addpath('./basic function');
% Extracts un-normalized twist (6x1 vector) from homogeneous transformation matrix (4x4)

R = g(1:3,1:3);
p = g(1:3,4);

theta = acos((trace(R) - 1) / 2);

if theta == 0
    w = [0;0;0];
    v = p;
    xi = [v;w];
else
    w_hat = 1/(2*sin(theta))*(R-R');
    w = [w_hat(3,2);w_hat(1,3);w_hat(2,1)];
    v = ((eye(3) - R)*w_hat+w*w'*theta) \ p;
    xi = theta*[v;w];
end
end