function g = twist(xi,theta)

w = xi(4:6);
v = xi(1:3);

w_hat = SKEW3(w);
I = eye(3);

Rotation = EXPCR(w*theta);
Translation = (I-Rotation)*w_hat*v+w*w'*v*theta;

g = [Rotation, Translation; 0 0 0 1];
