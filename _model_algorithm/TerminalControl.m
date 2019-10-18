function [Y_TC,cost_TC,P0_TC,S_list_TC] = TerminalControl(A,B,D,finite_time_of_DOC,Sw,xi,xf,Q,Qf,w,Y);

%% Stochastic Regulator

t_final = finite_time_of_DOC;
dt_are = 0.01;

%% solution of Lyapunove equation
% Qf_vector = Qf(:);
Sf_vector = zeros(49,1); %S(T) = 0

t_inverse = [finite_time_of_DOC : -dt_are : 0];

[T X] = ode45(@mLyapunov, t_inverse, Sf_vector, [], A, B, Q);
S_list_TC = X;

%P0
for i = 1 : length(A)
    for j = 1 : length(A)
        S0(i,j) = S_list_TC(end,j+(i-1)*length(A));
    end
end
P0_TC = inv(S0);

%% cost of stochastic regulation
cost_TC = 0;
for k = 1 : length(t_inverse)
    for i = 1 : length(A)
        for j = 1 : length(A)
            S(i,j) = X(k,j+(i-1)*length(A));            
        end
    end
    P = inv(S)
    dummy = trace(P*D*Sw*D')*dt_are;
    dummy_k(k) = trace(P*D*Sw*D')*dt_are;
    cost_TC = cost_TC + dummy;
end
cost_TC
% TC_cost = TC_cost + xi'*P0*xi;

%% state resoponse by stochastic regulator input
t_forward = [0 : dt_are : finite_time_of_DOC];

% xi = zeros(7,1);
% w = zeros(4,length(w));

Ts = 0;
Y_TC = xi';
for i = 1 : length(t_forward)-1
    S_temp = S_list_TC(end+1-i,:);
    S_temp = reshape(S_temp, size(A));
    P_temp = inv(S_temp);
    P_temp = P_temp(:);
    w_temp = w(:,i);
    [T_temp X_temp] = ode45(@state_space, [t_forward(i) t_forward(i+1)], Y_TC(end,:), [],A, B, D, P_temp,w_temp);
    Ts = [Ts T_temp(end)];
    Y_TC = [Y_TC; X_temp(end,:)];
end

%input of stochastic regulator
for i = 1 : length(t_forward)
    S_temp = S_list_TC(end+1-i,:);
    S_temp = reshape(S_temp, size(A));
    P_temp = inv(S_temp);
    us(:,i) = -B'*P_temp*Y_TC(i,:)';
end

% components of measure
sum_us = (sum(us(1,:).^2)+sum(us(2,:).^2))*dt_are
final = Y_TC(end,:)*Qf*Y_TC(end,:)'

% openloop response
sys = ss(A,D,eye(7,7),0);
[YY] = lsim(sys,[w],t_forward,xi);

%% plot - state resoponse
figure;plot(Ts,Y(:,1),'b','LineWidth',3)
hold on
plot(Ts,Y_TC(:,1),'--r','LineWidth',3)
plot(Ts,YY(:,1),'--k','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('time(sec)')
title('heave response')
legend('DODR','stochastic regulator','openloop','Location','SouthWest')
grid on
set(gcf,'Position', [1930 580 560 420])
xlim([t_final-0.5 t_final])

figure;plot(Ts,Y_TC(:,1)-YY(:,1),'--b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('time(sec)')
title('difference btw openloop and s.r')
grid on
set(gcf,'Position', [2510 580 560 420])
xlim([t_final-0.5 t_final])

figure;plot(Ts,us(1,:),'b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('time(sec)')
title('stochastic input')
grid on
set(gcf,'Position', [3090 50 560 420])
xlim([t_final-0.5 t_final])