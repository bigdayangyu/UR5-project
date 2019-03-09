function [angles] = EULERXYZINV(rot_matrix)
% For rotation matrices where the rotation around the X-axis is pi/2,
% EULERXYZ is ill-defined because we end up dividing by 0.

angles(1) = atan2(-rot_matrix(2,3), rot_matrix(3,3));

if (angles(1) == pi/2)
    disp('It looks like the rotation around the x-axis is pi/2 rads.');
    angles(2) = atan2(rot_matrix(1,3), -rot_matrix(2,3));
else
    angles(2) = atan2(rot_matrix(1,3), rot_matrix(3,3)/cos(angles(1)));
end

angles(3) = atan2(-rot_matrix(1,2), rot_matrix(1,1));

end

