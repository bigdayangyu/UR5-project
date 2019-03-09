function [totalRot] = EULERXYZ( angles )
% multiplies rotation matrices of X, Y, Z axis

totalRot = ROTX(angles(1)) * ROTY(angles(2)) * ROTZ(angles(3));

end