function [ pinvA ] = pseudoInverseRightMat(A, lambda)
    [m, ~] = size(A);
    pinvA = A'/(A*A'+lambda^2*eye(m));
end