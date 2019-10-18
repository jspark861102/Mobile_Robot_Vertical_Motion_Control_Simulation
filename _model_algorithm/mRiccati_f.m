function dXdt = mRiccati_f(t, X, A, B, Q)
X = reshape(X, size(A));                  %Convert from "n^2"-by-1 to "n"-by-"n"

%for forward
dXdt =1*(A'*X + X*A - X*B*B'*X + Q);       %Determine derivative

% %for backward
% dXdt =-1*(A'*X + X*A - X*B*B'*X + Q);       %Determine derivative


dXdt = dXdt(:);                           %Convert from "n"-by-"n" to "n^2"-by-1

