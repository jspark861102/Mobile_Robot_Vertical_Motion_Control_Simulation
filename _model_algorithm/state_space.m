function dxdt = state_space(t, x, A, B, D, P, w)

P = reshape(P, size(A));

dxdt = A*x -B*B'*P*x + D*w;      %Determine derivative

% count_ode = count_ode + 1;
% save count_ode.mat count_ode;