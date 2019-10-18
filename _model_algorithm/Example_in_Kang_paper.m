clear all
close all
clc


Ts=0.01;
Sw = 1;


%% system
An = [-1 -2 2 0 0 1;...
    0 -2 10 0 2 0;...
    0 0 -20 10 0 -20;...
    0 0 0 -3 0 0;...
    4 0 0 0 -10 2;...
    1 1 0 0 3 -15];

Bn = [1 -4 2;...
    5 1 0;...
    0 5 1;...
    0 1 0;...
    2 -4 0;...
    1 -2 0];

Cn = [0 0 0 0 1 0;...
    0 0 0 0 0 1];

Dn = [1 0;...
    0 2;...
    0 0;...
    0 0;...
    0 0;...
    0 0];

Bn = Bn(:,1:2);



%% another system
% k1 = 100; k2 = 200; k3 = 300;
% c1 = 0.1; c2 = 0.1; c3 = 0.1;
% m = 1;
% An = [0 0 0 1 0 0;...
%     0 0 0 0 1 0;...
%     0 0 0 0 0 1;...
%     -(k1+k2) k2 0 -(c1+c2) c2 0;...
%     k2 -(k2+k3) k3 c2 -(c2+c3) c3;...
%     0 k3 -k3 0 c3 -c3]
% 
% Bn = [0;0;0;0;0;1];
% 
% Dn = [0;0;0;1;0;0];

%%
[eigenvector,eigenvalue] = eig(An);
natural_frequency = abs(eigenvalue)/2/pi
damped_natural_frequency = abs(imag(eigenvalue))/2/pi
mode_shape = [eigenvector(:,1)/eigenvector(1,1) eigenvector(:,2)/eigenvector(1,2) eigenvector(:,3)/eigenvector(1,3) eigenvector(:,4)/eigenvector(1,4) eigenvector(:,5)];


%% by lyapunov
Wc_lyap = lyap(An,Bn*Bn');
Wd_lyap = lyap(An,Dn*Sw*Dn');

% rho_lyap = trace(inv(Wc_lyap)*Wd_lyap)
% rho_y_lyap = trace(inv(Cn*Wc_lyap*Cn')*(Cn*Wd_lyap*Cn'))


t_final=3;
f = 10; %352

% tau = 1/2/pi/f;
% sys_actuator = tf([1],[tau 1]);
% 
% A_actuator = [An Bn;zeros(2,length(An)) eye(2,2)*-1/tau];
% B_actuator = [zeros(6,2);eye(2,2)*1/tau];
% C_actuator = [Cn zeros(2,2)];
% D_actuator = [Dn;zeros(2,2)];   
% 
% 
% Wc_lyap_actuator = lyap(A_actuator,B_actuator*B_actuator');
% Wd_lyap_actuator = lyap(A_actuator,D_actuator*Sw*D_actuator');
% 
% % rho_y_lyap_actuator = trace(inv(C_actuator*Wc_lyap_actuator*C_actuator')*(C_actuator*Wd_lyap_actuator*C_actuator'))
% rho_y_lyap_actuator = trace(inv(Wc_lyap_actuator)*(Wd_lyap_actuator));

%% by ODE
tau = 1/2/pi/f;
sys_actuator = tf([1],[tau 1]);
A_actuator = [An Bn;zeros(size(Bn,2),length(An)) eye(size(Bn,2),size(Bn,2))*-1/tau];
B_actuator = [zeros(6,size(Bn,2));eye(size(Bn,2),size(Bn,2))*1/tau];
% C_actuator = [Cn zeros(size(Bn,2),size(Bn,2))];
D_actuator = [Dn;zeros(size(Bn,2),size(Bn,2))];  

A = A_actuator; B = B_actuator; D = D_actuator;   
save A.mat A;save B.mat B;      

%Wc
ini = zeros(1,length(A)^2);
[t,y] = ode45(@gra_AB,[0 :Ts: t_final],ini);
for i = 1 : length(A)
    for j = 1 : length(A)
        Wc(i,j) = y(end,j+(i-1)*length(A));
    end
end    
iWc = inv(Wc);

%Wd
Dc = D;
save Dc.mat;
save Sw.mat;
ini = zeros(1,length(A)^2);
[t,yd] = ode45(@gra_AD,[0 :Ts: t_final],ini);
for i = 1 : length(A)
    for j = 1 : length(A)
        Wd(i,j) = yd(end,j+(i-1)*length(A));
    end
end
rho = trace(iWc*(Wd))  


% w = 17.5*rand(length([0:Ts:t_final]),1)';
w = 1*rand(length([0:Ts:t_final]),size(Bn,2))';
% Fs=100;
% NFFT=64;%256;
% NOVERLAP=0.5*NFFT;
% WINDOW=hanning(NFFT);
% dt=1/Fs;
% dF=Fs/NFFT;
% 
% noise_f = zeros(NFFT,1);
% Szuf_zuf = zeros(NFFT,1);
% N=floor((length(w)-NFFT)/(NFFT-NOVERLAP))+1;
% for n=1:N
%     zuf_f=fft(w(1+(n-1)*(NFFT-NOVERLAP):NFFT+(n-1)*(NFFT-NOVERLAP)))'/NFFT;   
%     Szuf_zuf = Szuf_zuf + conj(zuf_f).*zuf_f/N/dF;  
% end
% 
% % Correlation function
% tau_corr = -(NFFT/Fs/2):dt:(NFFT/Fs/2)-dt;
% Rzuf_zuf = fftshift(ifft(Szuf_zuf,NFFT));
% max(abs(Rzuf_zuf))

u = zeros(size(Bn,2),length(t));
sys_dsum = ss(A,D,eye(length(A)),0);
[Y_dsum] = lsim(sys_dsum,w,[0:Ts:t_final],zeros(length(A),1));
dsum = Y_dsum(end,:)';

for i = 1 : length(t)    
    u(:,i) = B'*expm(A'*(t_final-t(i)))*iWc*(-dsum);
end
% if type/2 == 2
%     rho_from_input = (sum(u(1,:).^2)+sum(u(2,:).^2))*Ts;
% else
%     rho_from_input = sum(u(1,:).^2)*Ts;
% end

%response
sys = ss(A,[B D],eye(length(A)),0);
[Y] = lsim(sys,[u;w],t,zeros(length(A),1));

figure;
% set(gcf,'Position', [1950 550 630 450])
subplot(231)
plot(t,Y(:,1)','r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('m')
%     title('heave')
% legend('w/ consideration','w/o consideration','Location','South')
grid on

subplot(232)
plot(t,Y(:,2),'r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('[rad]')
% title('pitch')
% legend('w/ consideration','w/o consideration','Location','North')
grid on

subplot(233)
plot(t,Y(:,3),'r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('m/s')
% title('dx')
% legend('w/ consideration','w/o consideration','Location','NorthWest')
grid on

subplot(234)
plot(t,Y(:,4),'r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('m/s')
% title('heave velocity')
% legend('w/ consideration','w/o consideration','Location','NorthEast')
% set(gcf,'Position', [640 50 630 450])
grid on

subplot(235)
plot(t,Y(:,5),'r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('[rad/s]')
% title('pitch rate')
% legend('w/ consideration','w/o consideration','Location','SouthEast')
% set(gcf,'Position', [1270 50 630 450])
grid on

subplot(236)
plot(t,Y(:,6),'r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('[rad/s]')
% title('pitch rate')
% legend('w/ consideration','w/o consideration','Location','SouthEast')
% set(gcf,'Position', [1270 50 630 450])
grid on

figure;
plot(t,Y(:,7),'r','LineWidth',3)
% set(gca,'Fontsize',16,'FontWeight','demi')
xlabel('time(sec)')
ylabel('[rad/s]')
% title('pitch rate')
% legend('w/ consideration','w/o consideration','Location','SouthEast')
% set(gcf,'Position', [1270 50 630 450])
grid on

    


%% data set
% 
% t_final_list = [30];
% f_list = [0.5 : 0.5 :10];
% NE_rho_set2 = 0;
% NE_rho_y_lyap_actuator_set2 = 0;
% for p = 1 : length(t_final_list)
%     for k = 1 : length(f_list)
%         f = f_list(k);
%         tau = 1/2/pi/f;
%         sys_actuator = tf([1],[tau 1]);
%         A_actuator = [An Bn;zeros(size(Bn,2),length(An)) eye(size(Bn,2),size(Bn,2))*-1/tau];
%         B_actuator = [zeros(6,size(Bn,2));eye(size(Bn,2),size(Bn,2))*1/tau];
%         D_actuator = [Dn;zeros(size(Bn,2),size(Bn,2))];  
% 
%         Wc_lyap_actuator = lyap(A_actuator,B_actuator*B_actuator');
%         Wd_lyap_actuator = lyap(A_actuator,D_actuator*Sw*D_actuator');
%         rho_y_lyap_actuator = trace(inv(Wc_lyap_actuator')*(Wd_lyap_actuator))
%         NE_rho_y_lyap_actuator_set2 = [NE_rho_y_lyap_actuator_set2; rho_y_lyap_actuator];
% 
%         A = A_actuator; B = B_actuator; D = D_actuator;   
%         save A.mat A;save B.mat B;      
% 
%         %Wc
%         ini = zeros(1,length(A)^2);
%         [t,y] = ode45(@gra_AB,[0 :Ts: t_final_list(p)],ini);
%         for i = 1 : length(A)
%             for j = 1 : length(A)
%                 Wc(i,j) = y(end,j+(i-1)*length(A));
%             end
%         end    
%         iWc = inv(Wc);
% 
%         %Wd
%         Dc = D;
%         save Dc.mat;
%         save Sw.mat;
%         ini = zeros(1,length(A)^2);
%         [t,yd] = ode45(@gra_AD,[0 :Ts: t_final_list(p)],ini);
%         for i = 1 : length(A)
%             for j = 1 : length(A)
%                 Wd(i,j) = yd(end,j+(i-1)*length(A));
%             end
%         end
%         rho = trace(iWc*(Wd))  
%         NE_rho_set2 = [NE_rho_set2;rho];
% 
%         [p k]
%     end
% end
% NE_rho_set2 = NE_rho_set2(2:end);
% NE_rho_y_lyap_actuator_set2 = NE_rho_y_lyap_actuator_set2(2:end);
% 
% save NE_rho_set2.mat NE_rho_set2;
% save NE_rho_y_lyap_actuator_set2.mat NE_rho_y_lyap_actuator_set2;


%%
t_final_list = [1 : 1 : 30];
f_list = [0.5 : 0.5 :10];
% load NE_rho_y_lyap_actuator_set_withC.mat;
load NE_rho_y_lyap_actuator_set.mat;
load NE_rho_set.mat;
NE_rho_set_mat = zeros(30,20);
 for i = 1 : length(t_final_list)
    for j = 1 : length(f_list)
        NE_rho_set_mat(i,j) = NE_rho_set((i-1)*length(f_list) + j);        
    end
 end

% figure;plot(f_list,NE_rho_y_lyap_actuator_set,'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('DODR at infinite time')
% grid on

% for k = 1 : length(t_final_list)
%     figure;plot(f_list,NE_rho_y_lyap_actuator_set,'r','LineWidth',3)
%     hold on
%     plot(f_list,NE_rho_set_mat(k,:),'--b','LineWidth',3)
% end

% figure;plot(f_list,NE_rho_set_mat(end,:),'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('DODR at final time=30sec')
% grid on

figure;plot(f_list,NE_rho_y_lyap_actuator_set,'r','LineWidth',3)
hold on
plot(f_list,NE_rho_set_mat(end,:),'--b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
title('DODR')
legend('infinite time','final time=30sec')
grid on




% for q = 1 : length(f_list)
%     figure;plot(t_final_list,NE_rho_set_mat(:,q),'r','LineWidth',3)
%     grid on
% end

% figure;plot(f_list,NE_rho_y_lyap_actuator_set2,'r')
% hold on
% plot(f_list,NE_rho_set2,'--b')


