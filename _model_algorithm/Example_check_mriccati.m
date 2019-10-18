%%%%%%% 이 예제를 통해, Stochastic Regulator의 input 및 response를 구하는 과정(code)이 문제 없다는 것을 확인 하였다! %%%%%%%%%

clear all
close all
clc

f = 8;

% f_set = [0.5 : 0.5 : 10];
% cost_augmented_SR_set = null(1);
% cost_nominal_SR_set = null(1);
% for i = 1 : length(f_set)
% f = f_set(i);

tau = 1/2/pi/f;
% alpha = 10^13;
alpha = 10^10;

%% parameters : mass - ground
m = 10;
C = 800;
K = 5000;

% f = 1;
% tau = 1/2/pi/f;
% % alpha = 10^13;
% alpha = 10^9;

% nominal system
An = [0 1; -K/m -C/m];
Bn = [0;1/m];
Dn = [0;1000];

Qn = [0 0;0 0];
% Mn = alpha*[K 0;0 m]; 
Mn = alpha*[K 0;0 m]; 

xin = [0;0];

% actuator augmented system
A = [0 1 0; -K/m -C/m 1/m;0 0 -1/tau];
B = [0;0;1/tau];
D = [0;1000;0];

Q = zeros(3,3);
% M = alpha*[K 0 0;0 m 0;0 0 0]; 
M = alpha*[K 0 0;0 m 0;0 0 0/alpha]; 


xi = [0;0;0];

%% parameters : mass - spring - mass
% m1 = 10; m2 = 15;
% K=5000;
% C=200;
% 
% m_m = [m1 0;0 m2];
% K_m = [K -K;-K K];
% C_m = [C -C;-C C];
% 
% m = m1*m2/(m1+m2);
% 
% 
% % nominal system
% An = [0 0 1 0;0 0 0 1; -K/m1 K/m1 -C/m1 C/m1;K/m2 -K/m2 C/m2 -C/m2];
% Bn = [0;0;-1/m1;0];
% Dn = [0;0;0;1/m2];
% 
% Qn = zeros(4,4);
% Mn = alpha*[K -K 0 0;
%             -K K 0 0;
%             0 0 m1 0;
%             0 0 0 m2];
% 
% xin = [0;0;0;0];xfn = [0;0;0;0];
% 
% % actuator augmented system
% A = [An Bn;0 0 0 0 -1/tau];
% B = [0;0;0;0;1/tau];
% D = [Dn;0];
% 
% Q = zeros(5,5);
% M = alpha*[Mn/alpha zeros(4,1);zeros(1,4) 0]; 
% 
% xi = [0;0;0;0;0];xf = [0;0;0;0;0];

%% parameters : mass - spring - mass - spring - ground
% m1 = 10; m2 = 15;
% K=5000; C=100;
% K2 = 3500 ; C2 = 100;
% 
% m_m = [m1 0;0 m2];
% K_m = [K -K;-K K+K2];
% C_m = [C -C;-C C+C2];
% 
% m = m1*m2/(m1+m2);
% 
% % nominal system
% An = [0 0 1 0;0 0 0 1; -K/m1 K/m1 -C/m1 C/m1;K/m2 (-K-K2)/m2 C/m2 (-C-C2)/m2];
% Bn = [0;0;-1/m1;0];
% Dn = [0;0;0;1/m2];
% 
% Qn = zeros(4,4);
% Mn = alpha*[K -K 0 0;
%             -K K 0 0;
%             0 0 m1 0;
%             0 0 0 m2];
% 
% xin = [0;0;0;0]; xfn = [0;0;0;0];
% 
% % actuator augmented system
% A = [An Bn;0 0 0 0 -1/tau];
% B = [0;0;0;0;1/tau];
% D = [Dn;0];
% 
% Q = zeros(5,5);
% M = alpha*[Mn/alpha zeros(4,1);zeros(1,4) 0]; 
% 
% xi = [0;0;0;0;0]; xf = [0;0;0;0;0];

%% analysis
ctr = rank(ctrb(An,Bn))
% matched_condition = rank([Bn Dn])
% [eigenvector,eigenvalue] = eig(An);eigenvalue
% K_tilt = m_m^(-1/2)*K_m*m_m^(-1/2);
% natural_frequency = sqrt( eig(K_tilt) )/2/pi
% zeta = C/(2*sqrt(m*K))
% if zeta <= 1
%     damped_natural_frequency = natural_frequency*sqrt(1-zeta^2)
% else
%     damped_natural_frequency = null(length(K_tilt),length(K_tilt))
% end

%% solution
t_final = 3;
dt_are = 0.01;
t_inverse = [t_final : -0.01 : 0];
t = [0 : 0.01 : t_final];

%with Mn
Qfn_vector = Mn(:);
[Tn Xn] = ode45(@mRiccati, t_inverse, Qfn_vector, [], An, Bn, Qn);

P0n = reshape(Xn(end,:),size(An));
P0n_are = are(An,Bn*Bn',Qn); %independent for M

%with M
Qf_vector = M(:);
[T X] = ode45(@mRiccati, t_inverse, Qf_vector, [], A, B, Q);

P0 = reshape(X(end,:),size(A));
P0_are = are(A,B*B',Q); %independent for M

%% response
% load road_noise_rand_50.mat;
% load Sw.mat;
% w = road_noise_rand_50 - mean(road_noise_rand_50);
load w_rand_3.mat;
w = w_rand_3;

% road_noise = rand(length([0:dt_are:t_final]),1)';
% w = road_noise - mean(road_noise);

% Sw = Sw(1,1);
Sw = cov(w); %it's right!!
% Sw = std(w);

% nominal
sysn_openloop = ss(An,[Bn Dn],eye(size(An)),0);
[Yn_openloop] = lsim(sysn_openloop,[zeros(1,length(t)); w],t,xin);

sysn_ss = ss(An-Bn*Bn'*P0n_are,Dn,eye(size(An)),0);
[Yn_ss] = lsim(sysn_ss,w,t,xin);

Yn_SR = xin';
for i = 1 : length(t)-1
    Pn_SR = Xn(end+1-i,:);
    wn_SR = w(:,i);
    [Tn_SR Yn_SR_temp] = ode45(@state_space, [t(i) t(i+1)], Yn_SR(end,:), [],An, Bn, Dn, Pn_SR,wn_SR);        
    Yn_SR = [Yn_SR; Yn_SR_temp(end,:)];
end

for i = 1 : length(t)
    Pn_temp2 = Xn(end+1-i,:);
    Pn_temp2 = reshape(Pn_temp2, size(An));    
%     EIGENVALUEn=eig(Pn_temp2);
%     if EIGENVALUEn(1) < 0 || EIGENVALUEn(2) < 0 || EIGENVALUEn(3) < 0 || EIGENVALUEn(4) < 0 
%         EIGENVALUEn    
%     end
    un_SR(:,i) = -Bn'*Pn_temp2*Yn_SR(i,:)';
end

% augmented
sys_openloop = ss(A,[B D],eye(size(A)),0);
[Y_openloop] = lsim(sys_openloop,[zeros(1,length(t)); w],t,xi);

sys_ss = ss(A-B*B'*P0_are,D,eye(size(A)),0);
[Y_ss] = lsim(sys_ss,w,t,xi);

Y_SR = xi';
for i = 1 : length(t)-1
    P_SR = X(end+1-i,:);
    w_SR = w(:,i);
    [T_SR Y_SR_temp] = ode45(@state_space, [t(i) t(i+1)], Y_SR(end,:), [],A, B, D, P_SR,w_SR);        
    Y_SR = [Y_SR; Y_SR_temp(end,:)];
end

for i = 1 : length(t)
    P_temp2 = X(end+1-i,:);
    P_temp2 = reshape(P_temp2, size(A));  
%     EIGENVALUE = eig(P_temp2);
%     if EIGENVALUE(1) < 0 || EIGENVALUE(2) < 0 || EIGENVALUE(3) < 0 || EIGENVALUE(4) < 0 || EIGENVALUE(5) < 0
%         EIGENVALUE
%     end
    u_SR(:,i) = -B'*P_temp2*Y_SR(i,:)';
end

%% cost of stochastic regulation
% nominal
cost_nominal_SR = 0;
for k = 2 : length(t_inverse)                     %P(tf) 빼고 
    Pn = reshape(Xn(k,:),size(An));                
    dummyn = trace(Pn*Dn*Sw*Dn')*dt_are;
    dummyn_k(k) = trace(Pn*Dn*Sw*Dn')*dt_are;
    cost_nominal_SR = cost_nominal_SR + dummyn;
end
cost_nominal_SR
% dummyn_k(3)
% figure;plot(dummyn_k)

% components of measure
sum_un = sum(un_SR(1,1:end-1).^2)*dt_are
finaln = Yn_SR(end,:)*Mn*Yn_SR(end,:)'

% augmented
cost_augmented_SR = 0;
for k = 2 : length(t_inverse)                     %P(tf) 빼고
    P = reshape(X(k,:),size(A));                
    dummy = trace(P*D*Sw*D')*dt_are;
    dummy_k(k) = trace(P*D*Sw*D')*dt_are;
    cost_augmented_SR = cost_augmented_SR + dummy;
end
cost_augmented_SR
% figure;plot(dummy_k)

% components of measure
sum_u = sum(u_SR(1,1:end-1).^2)*dt_are
final = Y_SR(end,:)*M*Y_SR(end,:)'
% difference = cost_augmented_SR - sum_u - final
% divide = cost_augmented_SR/(sum_u + final)


% cost_nominal_SR_set = [cost_nominal_SR_set cost_nominal_SR];
% sum_un_set = [sum_un_set sum_un];
% finaln_set = [finaln_set finaln];
% 
% si = si + 1
% end
% cost_nominal_SR_set = cost_nominal_SR_set(2:end);
% sum_un_set = sum_un_set(2:end);
% finaln_set = finaln_set(2:end);
% mean(cost_nominal_SR_set)
% mean(sum_un_set)
% mean(finaln_set)

%%
% cost_nominal_SR_set = [cost_nominal_SR_set cost_nominal_SR]; 
% cost_augmented_SR_set = [cost_augmented_SR_set cost_augmented_SR];
% end
% figure;plot(f_set,cost_nominal_SR_set,'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% grid on
% set(gcf,'Position', [1930 580 560 420])
% 
% figure;plot(f_set,cost_augmented_SR_set,'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% grid on
% set(gcf,'Position', [1930 580 560 420])

%%
figure;plot(t,Yn_openloop(:,1),'k','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
plot(t,Yn_SR(:,1),'-.r','LineWidth',3)
xlabel('time(sec)')
title('response - x')
legend('openloop','closedloop-Stochastic Regulator','Location','South')
grid on
set(gcf,'Position', [1930 580 560 420])
% xlim([t_final-1 t_final])

figure;plot(t,Yn_openloop(:,2),'k','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
plot(t,Yn_SR(:,2),'-.r','LineWidth',3)
xlabel('time(sec)')
title('response - dx')
legend('openloop','closedloop-Stochastic Regulator','Location','South')
grid on
set(gcf,'Position', [2510 580 560 420])
% xlim([48 50])

% figure;plot(t(1:end),un_SR(1:end),'b','LineWidth',3)
figure;plot(t(1:end-1),un_SR(1:end-1),'b','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
xlabel('time(sec)')
title('input(u)')
grid on
set(gcf,'Position', [3090 580 560 420])
% xlim([48 50])
% ylim([-400 100])

figure;plot(t,Y_openloop(:,1),'k','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
plot(t,Y_SR(:,1),'-.r','LineWidth',3)
xlabel('time(sec)')
title('response - x')
legend('openloop','closedloop-Stochastic Regulator','Location','South')
grid on
set(gcf,'Position', [1930 50 560 420])
xlim([2 3])

figure;plot(t,Y_openloop(:,2),'k','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
plot(t,Y_SR(:,2),'-.r','LineWidth',3)
xlabel('time(sec)')
title('response - dx')
legend('openloop','closedloop-Stochastic Regulator','Location','South')
grid on
set(gcf,'Position', [2510 50 560 420])
xlim([2 3])

% figure;plot(t(1:end),u_SR(1:end),'b','LineWidth',3)
figure;plot(t(1:end-1),u_SR(1:end-1),'b','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
xlabel('time(sec)')
title('input(u)')
grid on
set(gcf,'Position', [3090 50 560 420])
% xlim([48 50])
xlim([2 3])

%% plot
% figure;plot(t_inverse,Xn(:,1),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,X(:,1),'--r','LineWidth',3)
% % plot(t_inverse,P0_are(1,1)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(1,1) of P')
% legend('Mn','M')
% grid on
% set(gcf,'Position', [1930 580 560 420])
% xlim([48 50])
% 
% figure;plot(t_inverse,X1(:,3),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,X2(:,3),'--r','LineWidth',3)
% plot(t_inverse,P0_are(1,2)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% % plot(t_inverse,X1(:,3)-X2(:,3),'k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(1,2) of P')
% legend('M=zeros','M=nonzero')
% grid on
% set(gcf,'Position', [2510 580 560 420])
% xlim([48 50])
% 
% figure;plot(t_inverse,X1(:,2),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,X2(:,2),'--r','LineWidth',3)
% plot(t_inverse,P0_are(2,1)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% % plot(t_inverse,X1(:,2)-X2(:,2),'k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(2,1) of P')
% legend('M=zeros','M=nonzero')
% grid on
% set(gcf,'Position', [1930 50 560 420])
% xlim([48 50])
% 
% figure;plot(t_inverse,X1(:,4),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,X2(:,4),'--r','LineWidth',3)
% plot(t_inverse,P0_are(2,2)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% % plot(t_inverse,X1(:,4)-X2(:,4),'k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(2,2) of P')
% legend('M=zeros','M=nonzero')
% grid on
% set(gcf,'Position', [2510 50 560 420])
% xlim([48 50])

% figure;plot(t_inverse,Xn(:,4)-X(:,4),'k','LineWidth',3)
