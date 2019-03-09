function skew = SKEW3(X)

% X is a 3x1 vector and skew is a 3x3 skew matrix of X

skew = [0, -X(3), X(2); X(3), 0, -X(1); -X(2), X(1), 0];