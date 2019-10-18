function [Y_SR,cost_SR,P0_SR,P_list_SR,sum_us,final_SR,us,YY] = StochasticRegulator(A,B,D,finite_time_of_DOC,Sw,xi,xf,Q,Qf,w,Y);

%% Stochastic Regulator

t_final = finite_time_of_DOC;
dt_are = 0.01;

xi = zeros(7,1);

%% solution of riccati equation
Qf_vector = Qf(:);
t_inverse = [finite_time_of_DOC : -dt_are : 0];

[T X] = ode45(@mRiccati, t_inverse, Qf_vector, [], A, B, Q);
P_list_SR = X;

%P0
for i = 1 : length(A)
    for j = 1 : length(A)
        P0(i,j) = X(end,j+(i-1)*length(A));
    end
end
P0_SR = P0;

%% cost of stochastic regulation
cost_SR = 0;
for k = 1 : length(t_inverse)
    for i = 1 : length(A)
        for j = 1 : length(A)
            P(i,j) = X(k,j+(i-1)*length(A));            
        end
    end
    dummy = trace(P*D*Sw*D')*dt_are;
    dummy_k(k) = trace(P*D*Sw*D')*dt_are;
    cost_SR = cost_SR + dummy;
end
cost_SR
% SR_cost = SR_cost + xi'*P0*xi;

%% plot - components of P(t)
% figure;plot(dummy_k,'b','LineWidth',3)
% figure;plot(cumsum(dummy_k))

% for i = 1 : length(A)    
%     figure;plot(t_inverse,X(:,i+7*(i-1)),'b','LineWidth',3)
%     set(gca,'Fontsize',16,'FontWeight','demi')
%     xlabel('time(sec)')
%     % ylabel('')
%     title(['(' num2str(i) ',' num2str(i) ') of P'])
%     grid on
%     set(gcf,'Position', [1930 580 560 420])
%     xlim([49 50])
% end
%% state resoponse by stochastic regulator input
t_forward = [0 : dt_are : finite_time_of_DOC];

% xi = zeros(7,1);
% w = zeros(4,length(w));

Ts = 0;
Y_SR = xi';
for i = 1 : length(t_forward)-1
    P_temp = P_list_SR(end+1-i,:);
    w_temp = w(:,i);
    [T_temp X_temp] = ode45(@state_space, [t_forward(i) t_forward(i+1)], Y_SR(end,:), [],A, B, D, P_temp,w_temp);    
    Ts = [Ts T_temp(end)];
    Y_SR = [Y_SR; X_temp(end,:)];
end

%input of stochastic regulator
for i = 1 : length(t_forward)
    P_temp = P_list_SR(end+1-i,:);
    P_temp = reshape(P_temp, size(A));
    us(:,i) = -B'*P_temp*Y_SR(i,:)';
end

% components of measure
sum_us = (sum(us(1,:).^2)+sum(us(2,:).^2))*dt_are
final_SR = Y_SR(end,:)*Qf*Y_SR(end,:)'
final = Y_SR(end,1:5)*Y_SR(end,1:5)'
Y_SR(end,1:5)

% openloop response
sys = ss(A,D,eye(7,7),0);
[YY] = lsim(sys,[w],t_forward,xi);

%% plot - state resoponse
figure;plot(Ts,Y(:,1),'b','LineWidth',3)
hold on
plot(Ts,Y_SR(:,1),'--r','LineWidth',3)
plot(Ts,YY(:,1),'--k','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('time(sec)')
title('heave response')
legend('DODR','stochastic regulator','openloop','Location','SouthWest')
grid on
set(gcf,'Position', [1930 580 560 420])
xlim([t_final-0.5 t_final])

figure;plot(Ts,Y(:,2),'b','LineWidth',3)
hold on
plot(Ts,Y_SR(:,2),'--r','LineWidth',3)
plot(Ts,YY(:,2),'--k','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('time(sec)')
title('pitch response')
legend('DODR','stochastic regulator','openloop','Location','SouthWest')
grid on
set(gcf,'Position', [2510 580 560 420])
xlim([t_final-0.5 t_final])

% figure;plot(Ts,Y_SR(:,1)-YY(:,1),'--b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('time(sec)')
% title('difference btw openloop and s.r')
% grid on
% set(gcf,'Position', [2510 580 560 420])
% xlim([t_final-0.5 t_final])

figure;plot(Ts,us(1,:)+us(2,:),'b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('time(sec)')
title('stochastic input')
grid on
set(gcf,'Position', [3090 580 560 420])
xlim([t_final-0.5 t_final])