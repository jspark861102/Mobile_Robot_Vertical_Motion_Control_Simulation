function dXdt = mLyapunov(t, X, A, B, Q)
X = reshape(X, size(A));                  %Convert from "n^2"-by-1 to "n"-by-"n"

%for backward
dXdt = (X*A' + A*X - B*B');       %Determine derivative
% dXdt = X*A' + A*X + B*B';       %Determine derivative

dXdt = dXdt(:);                           %Convert from "n"-by-"n" to "n^2"-by-1
