function dXdt = gra_AB_modified(t, X, A, B)
X = reshape(X, size(A)); %Convert from "n^2"-by-1 to "n"-by-"n"
dXdt = A*X + X*A' + B*B'; %Determine derivative
% dXdt = A*X + X*A' + B*B'; %Determine derivative
dXdt = dXdt(:); %Convert from "n"-by-"n" to "n^2"-by-1

%% 이함수 사용할 때 쓸 code
% [T X] = ode45(@gra_AB_modified, [0 :Ts: t_final], ini, [], A, D_actuator);
% 
% [m n] = size(X);
% XX = mat2cell(X, ones(m,1), n);
% fh_reshape = @(x)reshape(x,size(A));
% XX = cellfun(fh_reshape,XX,'UniformOutput',false);