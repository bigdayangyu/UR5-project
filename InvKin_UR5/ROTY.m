function ry = ROTY(radians)

% compute sin and cos of this angle
s = sin(radians);
c = cos(radians);

% form the rotation matrix with x-axis
ry = [c 0 s; 0 1 0; -s 0 c];