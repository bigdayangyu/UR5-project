function rotation = EXPCR(X)

% X is a 3x1 vector represent rotation axis n and angle theta

theta = norm(X);

% if X is a zero vector rotation should be an identity matrix
if theta == 0
    rotation = eye(3,3);
else
    n = X / theta;

    % compute exp(n_hat * theta)
    n_hat = SKEW3(n);
    rotation = expm(n_hat * theta);
end