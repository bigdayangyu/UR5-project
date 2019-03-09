function rx = ROTX(radians)

% compute sin and cos of this angle
s = sin(radians);
c = cos(radians);

% form the rotation matrix with x-axis
rx = [1 0 0; 0 c -s; 0 s c];
