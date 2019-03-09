function rz = ROTZ(radians)

% compute sin and cos of this angle
s = sin(radians);
c = cos(radians);

% form the rotation matrix with x-axis
rz = [c -s 0; s c 0; 0 0 1];