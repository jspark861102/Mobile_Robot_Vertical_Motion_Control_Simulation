%%%%%%% 이 예제를 통해, Stochastic Regulator의 input 및 response를 구하는 과정(code)이 문제 없다는 것을 확인 하였다! %%%%%%%%%

clear all
close all
clc

f = 10;

% f_set = [0.5 : 0.5 : 10];
% cost_augmented_LV_set = null(1);
% cost_nominal_LV_set = null(1);
% for i = 1 : length(f_set)
% f = f_set(i);
    

tau = 1/2/pi/f;

%% parameters : mass - ground
m =10;
C = 820;
K = 5000;

% nominal system
An = [0 1; -K/m -C/m];
Bn = [0;1/m];
Dn = [0;1000];

Qn = [0 0;0 0];
Sfn= [0 0;0 0];

xin = [0;0];

% actuator augmented system
A = [0 1 0; -K/m -C/m 1/m;0 0 -1/tau];
B = [0;0;1/tau];
D = [0;1000;0];

Q = [0 0 0;0 0 0;0 0 0];
Sf = [0 0 0;0 0 0;0 0 0];

xi = [0;0;0];

%% parameters : mass - spring - mass
% m1 = 5; m2 = 5;
% K=100;
% C=100;
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
% Qn = [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
% Sfn= [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
% 
% xin = [0;0;0;0];xfn = [0;0;0;0];
% 
% % actuator augmented system
% A = [An Bn;0 0 0 0 -1/tau];
% B = [0;0;0;0;1/tau];
% D = [Dn;0];
% 
% Q = [0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0];
% Sf= [0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0];
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
% Qn = [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
% Sfn= [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
% 
% xin = [0;0;0;0]; xfn = [0;0;0;0];
% 
% % actuator augmented system
% A = [An Bn;0 0 0 0 -1/tau];
% B = [0;0;0;0;1/tau];
% D = [Dn;0];
% 
% Q = [0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0];
% Sf= [0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0];
% 
% xi = [0;0;0;0;0]; xf = [0;0;0;0;0];

%% analysis
% ctr = rank(ctrb(An,Bn));                                                                                              %%f가 커지면 uncontrollable해진다
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% nominal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_final = 3;
dt_are = 0.01;
t_inverse = [t_final : -0.01 : 0];
t = [0 : 0.01 : t_final];

%with Sfn
Qfn_vector = Sfn(:);
[Tn Xn] = ode45(@mLyapunov, t_inverse, Qfn_vector, [], An, Bn, Qn); 
Xn(isnan(Xn))=[1e290]; %너무 커져서 nan(infinte)가 되는 값들을 limit시키는 과정

% S0n = reshape(Xn(end,:),size(An));eig(S0n);rank(S0n);
% P0n = inv(S0n);
% S0n_are = lyap(An,-Bn*Bn');
% P0n_are = inv(S0n_are); 

Pn_list = zeros(1,length(xin)^2);
for k = 1 : length(t_inverse) 
    Sn_temp = Xn(k,:);
    Sn_temp2 = reshape(Sn_temp,size(An));
    if det(Sn_temp2) > 1e290 %inverse 구하는 과정에서 Sn이 너무 커서 infinite이면 임의로 0으로 만드는 과정
        Pn_temp = zeros(size(An));
    else        
        Pn_temp = inv(Sn_temp2);
    end
    Pn_temp2 = Pn_temp(:)';
    Pn_list = [Pn_list;Pn_temp2];
end
Pn_list = Pn_list(2:end,:);
Pn_list(isinf(Pn_list))=[0];
Pn_list(1,:) = inf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% augmented %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%with Sf
Qf_vector = Sf(:);
[T X] = ode45(@mLyapunov, t_inverse, Qf_vector, [], A, B, Q);
X(isnan(X))=[1e290];

% S0 = reshape(X(end,:),size(A));
% P0 = inv(S0);

% S0_are = lyap(A,-B*B');                                                                                              %%solution이 이상하다. 발산해야 하는데 수렴한다.
% P0_are = inv(S0_are); 

P_list = zeros(1,length(xi)^2);
for k = 1 : length(t_inverse) 
    S_temp = X(k,:);
    S_temp2 = reshape(S_temp,size(A));    
    if det(S_temp2) > 1e290
        P_temp = zeros(size(A));
    else        
        P_temp = inv(S_temp2);
    end
    P_temp2 = P_temp(:)';
    P_list = [P_list;P_temp2];
end
P_list = P_list(2:end,:);
P_list(isinf(P_list))=[0]; %% 임의로 1e290만든 term은 inverse시 inf로 나오기 때문에 0으로 만드는 과정, 정상 범위에서 inf값이 나오는 경우도 있는데 이것도 일단은 0으로 같이 만드는 과정
P_list(1,:) = inf;

%% noise
% load road_noise_rand_50.mat;
% w = road_noise_rand_50 - mean(road_noise_rand_50);
load w_rand_3.mat;
w = w_rand_3;

% road_noise = rand(length([0:dt_are:t_final]),1)';
% w = road_noise - mean(road_noise);

Sw = cov(w); %it's right!!

%% response
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% nominal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nominal
sysn_openloop = ss(An,[Bn Dn],eye(size(An)),0);
[Yn_openloop] = lsim(sysn_openloop,[zeros(1,length(t)); w],t,xin);

% sysn_ss = ss(An-Bn*Bn'*P0n_are,Dn,eye(size(An)),0);
% [Yn_ss] = lsim(sysn_ss,w,t,xin);

Yn_LV = xin';
for i = 1 : length(t)-1
    Sn_temp1 = Xn(end+1-i,:);
    Sn_temp2 = reshape(Sn_temp1,size(An));    
    if det(Sn_temp2) > 1e290
        Pn_temp1 = zeros(size(An));
    else        
        Pn_temp1 = inv(Sn_temp2);
    end    
    Pn_temp2 = Pn_temp1(:);
    Pn_temp2(isinf(Pn_temp2))=[0];    
    
    wn_LY = w(:,i);
    [Tn_LV Yn_LV_temp] = ode45(@state_space, [t(i) t(i+1)], Yn_LV(end,:), [],An, Bn, Dn, Pn_temp2,wn_LY);    
    Yn_LV = [Yn_LV; Yn_LV_temp(end,:)];
end

%input of terminal control
for i = 1 : length(t)
    Sn_temp3 = Xn(end+1-i,:);
    Sn_temp4 = reshape(Sn_temp3, size(An));    
    if det(Sn_temp4) > 1e290
        Pn_temp3 = zeros(size(An));
    else        
        Pn_temp3 = inv(Sn_temp4);
    end
    if i ~= length(t)
        Pn_temp3(isinf(Pn_temp3))=[0];
    end
            
    un_LV(:,i) = -Bn'*Pn_temp3*Yn_LV(i,:)';
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% augmented %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% augmented
sys_openloop = ss(A,[B D],eye(size(A)),0);
[Y_openloop] = lsim(sys_openloop,[zeros(1,length(t)); w],t,xi);

% sys_ss = ss(A-B*B'*P0_are,D,eye(size(A)),0);
% [Y_ss] = lsim(sys_ss,w,t,xi);

Y_LV = xi';
for i = 1 : length(t)-1
    S_temp1 = X(end+1-i,:);
    S_temp2 = reshape(S_temp1,size(A));    
    if det(S_temp2) > 1e290
        P_temp1 = zeros(size(A));
    else        
        P_temp1 = inv(S_temp2);
    end    
    P_temp2 = P_temp1(:);
    P_temp2(isinf(P_temp2))=[0];
    
    w_LY = w(:,i);
    [T_LV Y_LV_temp] = ode45(@state_space, [t(i) t(i+1)], Y_LV(end,:), [],A, B, D, P_temp2,w_LY);  
    Y_LV = [Y_LV; Y_LV_temp(end,:)];
end

%input of terminal control
for i = 1 : length(t)
    S_temp3 = X(end+1-i,:);
    S_temp4 = reshape(S_temp3, size(A));
    if det(S_temp2) > 1e290
        P_temp3 = zeros(size(A));
    else        
        P_temp3 = inv(S_temp4);
    end
    if i ~= length(t)
        P_temp3(isinf(P_temp3))=[0];
    end    
    
    u_LV(:,i) = -B'*P_temp3*Y_LV(i,:)';
end 
%% cost of stochastic regulation
% nominal
cost_nominal_LV = 0;
for k = 2 : length(t_inverse)    
    Sn = reshape(Xn(k,:),size(An));    
    if det(Sn) > 1e290
        Pn = zeros(size(An));
    else        
        Pn = inv(Sn);
    end      
    Pn(isinf(Pn))=[0];
    
    dummyn = trace(Pn*Dn*Sw*Dn')*dt_are;
    dummyn_k(k) = trace(Pn*Dn*Sw*Dn')*dt_are;
    cost_nominal_LV = cost_nominal_LV + dummyn;
end
cost_nominal_LV

% components of measure
sum_un = sum(un_LV(1,1:end-1).^2)*dt_are
% finaln = Yn_LV(end,:)*inv(Sfn)*Yn_LV(end,:)'

% augmented
cost_augmented_LV = 0;
for k = 1 : length(t_inverse)    
    S = reshape(X(k,:),size(A));    
    if det(S) > 1e290
        P = zeros(size(A));
    else        
        P = inv(S);
    end  
    P(isinf(P))=[0];
   
    dummy = trace(P*D*Sw*D')*dt_are;
    dummy_k(k) = trace(P*D*Sw*D')*dt_are;
    cost_augmented_LV = cost_augmented_LV + dummy;
end
cost_augmented_LV

% components of measure
sum_u = sum(u_LV(1,1:end-1).^2)*dt_are
% final = Y_LV(end,:)*inv(Sf)*Y_LV(end,:)'

%%
% cost_nominal_LV_set = [cost_nominal_LV_set cost_nominal_LV]; 
% cost_augmented_LV_set = [cost_augmented_LV_set cost_augmented_LV];
% end
% figure;plot(f_set,cost_nominal_LV_set,'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% grid on
% set(gcf,'Position', [1930 580 560 420])
% 
% figure;plot(f_set,cost_augmented_LV_set,'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% grid on
% set(gcf,'Position', [1930 580 560 420])


%%
% % % figure;plot(t,Yn_openloop(:,1),'k','LineWidth',3)
% % % set(gca,'Fontsize',16,'FontWeight','demi')
% % % hold on
% % % plot(t,Yn_LV(:,1),'-.r','LineWidth',3)
% % % % plot(t,Yn_ss(:,1),'-.r','LineWidth',3)
% % % xlabel('time(sec)')
% % % title('response - x')
% % % legend('openloop','closedloop-lyapunov')
% % % grid on
% % % set(gcf,'Position', [1930 580 560 420])
% % % % xlim([t_final-1 t_final])
% % % 
% % % figure;plot(t,Yn_openloop(:,2),'k','LineWidth',3)
% % % set(gca,'Fontsize',16,'FontWeight','demi')
% % % hold on
% % % plot(t,Yn_LV(:,2),'-.r','LineWidth',3)
% % % xlabel('time(sec)')
% % % title('response - dx')
% % % legend('openloop','closedloop-lyapunov')
% % % grid on
% % % set(gcf,'Position', [2510 580 560 420])
% % % % xlim([48 50])
% % % 
% % % figure;plot(t(1:end-1),un_LV(1:end-1),'b','LineWidth',3)
% % % set(gca,'Fontsize',16,'FontWeight','demi')
% % % hold on
% % % xlabel('time(sec)')
% % % title('input(u)')
% % % grid on
% % % set(gcf,'Position', [3090 580 560 420])
% % % % xlim([48 50])
% % % % ylim([-400 100])



figure;plot(t,Y_openloop(:,1),'k','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
plot(t,Y_LV(:,1),'-.r','LineWidth',3)
% plot(t,Yn_ss(:,1),'-.r','LineWidth',3)
xlabel('time(sec)')
title('response - x')
legend('openloop','closedloop-lyapunov')
grid on
set(gcf,'Position', [1930 50 560 420])
xlim([2 3])

figure;plot(t,Y_openloop(:,2),'k','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
plot(t,Y_LV(:,2),'-.r','LineWidth',3)
xlabel('time(sec)')
title('response - dx')
legend('openloop','closedloop-lyapunov')
grid on
set(gcf,'Position', [2510 50 560 420])
xlim([2 3])

figure;plot(t(1:end-1),u_LV(1:end-1),'b','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
hold on
xlabel('time(sec)')
title('input(u)')
grid on
set(gcf,'Position', [3090 50 560 420])
xlim([2 3])

%% plot
% figure;plot(t_inverse,P_list(:,1),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,P0_are(1,1)*ones(1,length(t_inverse)),'--r','LineWidth',3)
% plot(t_inverse,P_list(:,1)' - P0_are(1,1)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(1,1) of P')
% legend('finited time','steady state')
% grid on
% set(gcf,'Position', [1930 580 560 420])
% xlim([48 50])
% 
% figure;plot(t_inverse,P_list(:,3),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,P0_are(1,2)*ones(1,length(t_inverse)),'--r','LineWidth',3)
% plot(t_inverse,P_list(:,3)' - P0_are(1,2)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(1,2) of P')
% legend('finited time','steady state')
% grid on
% set(gcf,'Position', [2510 580 560 420])
% xlim([48 50])
% 
% figure;plot(t_inverse,P_list(:,2),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,P0_are(2,1)*ones(1,length(t_inverse)),'--r','LineWidth',3)
% plot(t_inverse,P_list(:,2)' - P0_are(2,1)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(2,1) of P')
% legend('finited time','steady state')
% grid on
% set(gcf,'Position', [1930 50 560 420])
% xlim([48 50])
% 
% figure;plot(t_inverse,P_list(:,4),'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% hold on
% plot(t_inverse,P0_are(2,2)*ones(1,length(t_inverse)),'--r','LineWidth',3)
% plot(t_inverse,P_list(:,4)' - P0_are(2,2)*ones(1,length(t_inverse)),'--k','LineWidth',3)
% xlabel('time(sec)')
% % ylabel('')
% title('(2,2) of P')
% legend('finited time','steady state')
% grid on
% set(gcf,'Position', [2510 50 560 420])
% xlim([48 50])