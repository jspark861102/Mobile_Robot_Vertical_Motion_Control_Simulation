clear all
close all
clc

%% data acquisition_start 
% f_set = [0.5 : 0.5 : 20];
% 
% finalstate = null(1,5);
% finalstate_r = null(1,5);
% rho_set = null(1,1);
% rhor_set = null(1,1);
% 
% for i = 1 : length(f_set)
%% design parameters
K=5000;
C=800;
% C=2*sqrt(10*K)


% f = f_set(i)
% f = 100;
f = 1;
t_final = 3;
% alpha = 10^13;

tau = 1/2/pi/f;
%% parameters : mass - spring - ground system7
m = 10;

m_m = m;
K_m = K;
C_m = C;

% nominal system
Ar = [0 1; -K/m -C/m];    
Br = [0;1/m];
Dr = [0;1/m];

% Qn = [0 0;0 0];
% % Mn = alpha*[K 0;0 m]; 
% Mn = alpha*[K 0;0 m]; 

xir = [0;0];xfr = [0;0];

% actuator augmented system
A = [0 1 0; -K/m -C/m 1/m;0 0 -1/tau];
B = [0;0;1/tau];  
D = [0;1/m;0];

% Q = zeros(3,3);
% % M = alpha*[K 0 0;0 m 0;0 0 0]; 
% M = alpha*[K 0 0;0 m 0;0 0 0/alpha]; 

xi = [0;0;0];xf = [0;0;0];

%% parameters : mass - spring - mass
% m1 = 10; m2 = 15;
% 
% m_m = [m1 0;0 m2];
% K_m = [K -K;-K K];
% C_m = [C -C;-C C];
% 
% m = m1*m2/(m1+m2);
% 
% % nominal system
% Ar = [0 0 1 0;0 0 0 1; -K/m1 K/m1 -C/m1 C/m1;K/m2 -K/m2 C/m2 -C/m2];
% Br = [0;0;-1/m1;0];
% Dr = [0;0;0;1/m2];
% 
% % Qn = zeros(4,4);
% % Mn = alpha*[K -K 0 0;
% %             -K K 0 0;
% %             0 0 m1 0;
% %             0 0 0 m2];
% 
% xir = [0;0;0;0];xfr = [0;0;0;0];
% 
% % actuator augmented system
% A = [Ar Br;0 0 0 0 -1/tau];
% B = [0;0;0;0;1/tau];
% D = [Dr;0];
% 
% % Q = zeros(5,5);
% % M = alpha*[Mn/alpha zeros(4,1);zeros(1,4) 0]; 
% 
% xi = [0;0;0;0;0];xf = [0;0;0;0;0];

%% parameters : mass - spring - mass - spring - ground
% m1 = 10; m2 = 15;
% K2 = 3500 ; C2 = 200;
% m_m = [m1 0;0 m2];
% K_m = [K -K;-K K+K2];
% C_m = [C -C;-C C+C2];
% 
% m = m1*m2/(m1+m2);
% 
% % nominal system
% Ar = [0 0 1 0;0 0 0 1; -K/m1 K/m1 -C/m1 C/m1;K/m2 (-K-K2)/m2 C/m2 (-C-C2)/m2];
% Br = [0;0;-1/m1;0];
% Dr = [0;0;0;1/m2];
% 
% % Qn = zeros(4,4);
% % Mn = alpha*[K -K 0 0;
% %             -K K 0 0;
% %             0 0 m1 0;
% %             0 0 0 m2];
% 
% xir = [0;0;0;0];xfr = [0;0;0;0];
% 
% % actuator augmented system
% A = [Ar Br;0 0 0 0 -1/tau];
% B = [0;0;0;0;1/tau];
% D = [Dr;0];
% 
% % Q = zeros(5,5);
% % M = alpha*[Mn/alpha zeros(4,1);zeros(1,4) 0]; 
% 
% xi = [0;0;0;0;0];xf = [0;0;0;0;0];

%% analysis
% ctr = rank(ctrb(Ar,Br))
% matched_condition = rank([Br Dr])
% [eigenvector,eigenvalue] = eig(Ar);eigenvalue
% K_tilt = m_m^(-1/2)*K_m*m_m^(-1/2);
% natural_frequency = sqrt( eig(K_tilt) )/2/pi
% zeta = C/(2*sqrt(m*K))
% if zeta <= 1
%     damped_natural_frequency = natural_frequency*sqrt(1-zeta^2)
% else
%     damped_natural_frequency = null(length(K_tilt),length(K_tilt))
% end

%% response
dt_are = 0.01;
t = [0 : 0.01 : t_final];

load w_rand_3.mat;
w = w_rand_3;

% load w_rand_10.mat;
% w = w_rand_10;

% road_noise = rand(length([0:dt_are:t_final]),1)';
% w = road_noise - mean(road_noise);
% figure;plot(w)
Sw = cov(w); %it's right!!
% Sw = 0.0011; %it's right!!
% WWd = dsum*dsum'
% pinv(D)*(-A*WWd-WWd*A')*pinv(D') 

% nominal
sysn_openloop = ss(Ar,[Br Dr],eye(size(Ar)),0);
[Yn_openloop] = lsim(sysn_openloop,[zeros(1,length(t)); w],t,xir);

%%
save A.mat; save B.mat; save D.mat;
save Ar.mat; save Br.mat; save Dr.mat;
save Sw.mat;

[Wd,Wc,Wcr,iWc,iWcr,rho,rho_rigid,t,tr] = DOCandDODR(A,B,D,Ar,Br,Dr,t_final,dt_are,Sw,xi,xf,xir,xfr);
[rho_from_input,rhon_from_input,u,ur,Y,Yr] = InputandResponse1(iWc,iWcr,A,B,D,Ar,Br,Dr,t,tr,dt_are,xi,xf,xir,xfr,w,t_final);
rho_from_input
rhon_from_input
%% data acquisition_end
% finalstate = [finalstate ; Y(end,:)];
% finalstate_r = [finalstate_r ; Yr(end,:)];
% rho_set = [rho_set ; rho];
% rhor_set = [rhor_set ; rho_rigid];
% 
% end

%%
figure;
set(gcf,'Position', [1930 580 560 420])
plot(t,Yn_openloop(:,1)','k','LineWidth',2)
hold on
plot(t,Yr(:,1)','--b','LineWidth',3)
plot(tr,Y(:,1)','--r','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('m')
title('x')
legend('openloop','DODR-nominal','DODR-Acdayn','Location','SouthWest')
grid on

figure;
set(gcf,'Position', [2510 580 560 420])
plot(t,Yn_openloop(:,2)','k','LineWidth',2)
hold on
plot(t,Yr(:,2)','--b','LineWidth',3)
plot(tr,Y(:,2)','--r','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('m')
title('dx')
legend('openloop','DODR-nominal','DODR-Acdayn','Location','SouthWest')
grid on

figure;
set(gcf,'Position', [3090 580 560 420])
plot(t,ur,'b','LineWidth',3)
hold on
plot(tr,u,'--r','LineWidth',3)
set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('m/s')
title('u')
legend('DODR-nominal','DODR-Acdayn','Location','SouthWest')
grid on
% xlim([2 3])

% figure;
% set(gcf,'Position', [3090 50 560 420])
% plot(t,Yr(:,5),'b','LineWidth',3)
% hold on
% plot(tr,Y(:,5),'--r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('time(sec)')
% ylabel('m/s')
% title('u')
% legend('openloop','DODR','Location','NorthWest')
% grid on

%%
% figure;
% set(gcf,'Position', [1930 580 560 420])
% plot(f_set,finalstate(:,1),'b','LineWidth',3)
% % plot(f_set,(finalstate(:,4)+finalstate(:,4))/2,'b','LineWidth',3)
% hold on
% plot(f_set,finalstate_r(:,1),'--r','LineWidth',3)
% % plot(f_set,zeros(1,length(f_set)),'k','LineWidth',1)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('f[Hz]')
% % ylabel('m')
% title('finalstate-Y(1)')
% % legend('openloop','DODR','Location','NorthWest')
% grid on
% 
% figure;
% set(gcf,'Position', [2510 580 560 420])
% plot(f_set,finalstate(:,2),'b','LineWidth',3)
% hold on
% plot(f_set,finalstate_r(:,2),'--r','LineWidth',3)
% % plot(f_set,zeros(1,length(f_set)),'k','LineWidth',1)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('f[Hz]')
% % ylabel('m')
% title('finalstate-Y(2)')
% % legend('openloop','DODR','Location','NorthWest')
% grid on
% 
% figure;
% set(gcf,'Position', [3090 580 560 420])
% plot(f_set,rhor_set,'--k','LineWidth',3)
% hold on
% plot(f_set,rho_set,'b','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
% xlabel('f[Hz]')
% title('DODR')
% legend('nominal','augmentation','Location','NorthEast')
% grid on
