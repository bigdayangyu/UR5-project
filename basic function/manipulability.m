function mu = manipulability(J,mode)
% Computes a measure of manipulability. 
% Inputs J (6x6 Jacobian matrix) and mode (strings 'sigmamin', 'detjac',
% or 'invcond') outputs the corresponding measure of manipulability

% sigmamin computes the minimum singular value of J, which corresponds to
% the minimum workspace velocity produced by a unit joint velocity vector

% invcond calculates the ratio of the max singular value of J to the min
% singular value of J

% detjac computes the determinant of the Jacobian, which measures the
% volume of the velocity ellipsoid

[u,s,v] = svd(J);
mu1 = min(max(s));
mu2 = min(max(s))/max(max(s));
mu3 = det(s);

switch mode
    case 'sigmamin'
    mu = mu1;
    case 'invcond'
    mu = mu2;
    case 'detjac'
    mu = mu3;
    otherwise
    error('wrong mode! plz double-check!');
end
