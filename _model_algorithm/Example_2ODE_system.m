% simple exaple의 결과는, 구동기의 동특성이 느리면 에너지가 많이 들 뿐, 제어가 안되고 그런 상황은 아니다.
% 자동차 예제와는 다르다.

clear all
close all
clc

m = 9;
c = 23;
k = 127;

f = [0.1 : 0.1 : 10];
tau = 1/2/pi./f;
% tau = [0.1 : 0.1 : 10];

An = [0 1; -k/m -c/m];
Bn = [0;1/m];
Dn = [1.7;13];
Sw = 0.3;
[eigenvector,eigenvalue] = eig(An);
natural_frequency = abs(eigenvalue)/2/pi
damped_natural_frequency = abs(imag(eigenvalue))/2/pi

for i = 1 : length(tau)    
    Au = [An Bn;zeros(1,2) -1/tau(i)];
    Bu = [zeros(2,1);1/tau(i)];
    Du = [Dn;0];
    ctest(i) = rank(ctrb(Au,Bu));
    Wc_set(i,:,:)=lyap(An,Bn*Bn');
    Wcu_set(i,:,:)=lyap(Au,Bu*Bu');
    Wdu_set(i,:,:)=lyap(Au,Du*Sw*Du');
end

for j = 1 : 2
    for i = 1 : length(tau)    
        dummy(:,:) = Wc_set(i,:,:);
        idummy = inv(dummy);
        invWc_diagonal(j,i) = idummy(j,j);        
    end
end

for j = 1 : 3
    for i = 1 : length(tau)    
        dummy2(:,:) = Wcu_set(i,:,:);
        dummy4(:,:) = Wdu_set(i,:,:);
        idummy2 = inv(dummy2);
        invWcu_diagonal(j,i) = idummy2(j,j);       
        DODRm(i,:,:) = idummy2 * dummy4;
        DODR(i) = trace(idummy2 * dummy4);
    end
end

figure;plot(f,invWc_diagonal(1,:),'b','LineWidth',3)
hold on
plot(f,invWcu_diagonal(1,:),'--r','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
title('(1,1) of inverse of Wc and Wcu')
legend('iWc','iWcu')
grid on
% dummy11 = find(invWcu_diagonal(1,:) == max(invWcu_diagonal(1,:)));
% f11 = f(dummy11)

figure;plot(f,invWc_diagonal(2,:),'b','LineWidth',3)
hold on
plot(f,invWcu_diagonal(2,:),'--r','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
title('(2,2) of inverse of Wc and Wcu')
legend('iWc','iWcu')
grid on
% dummy22 = find(invWcu_diagonal(2,:) == max(invWcu_diagonal(2,:)));
% f22 = f(dummy22)

figure;plot(f,invWcu_diagonal(3,:),'--r','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
title('(3,3) of inverse of Wc and Wcu')
legend('iWcu')
grid on


figure;plot(f,DODR,'b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
title('DODR')
grid on

%%
for i = 1 : length(tau)
    p(i) = k*(tau(i)^2) + c*tau(i) + m;
%     Wcu_set_analytic(i,:,:) = [1/2/c/k 0 tau(i)/p(i);
%                     0 1/2/m/c 1/p(i);
%                     tau(i)/p(i) 1/p(i) 1/2/tau(i)];
%     iWcu_set_adj_analytic(i,:,:) = [(1/tau(i))*(1/4/m/c)-(1/p(i)/p(i)) tau(i)/p(i)/p(i) -(tau(i)/p(i))*(1/2/m/c);
%                      tau(i)/p(i)/p(i) (1/tau(i))*(1/4/c/k)-tau(i)*tau(i)/p(i)/p(i) (-(1/p(i))*(1/2/c/k));
%                      -(tau(i)/p(i))*(1/2/m/c) -(1/p(i))*(1/2/c/k) 1/2/m/c/c/k];
    Wcu_set_analytic(i,:,:) = [(m+c*tau(i))/(2*c*k)/p(i) 0 tau(i)/2/p(i);
                    0 1/2/c/p(i) 1/2/p(i);
                    tau(i)/2/p(i) 1/2/p(i) 1/2/tau(i)];            
    iWcu_set_adj_analytic(i,:,:) = [(p(i)/4/c/tau(i))-1/4 tau(i)/4 -tau(i)/4/c;
                     tau(i)/4 (m+c*tau(i))/(4*k*c*tau(i))*p(i)-tau(i)*tau(i)/4 -(m+c*tau(i))/(4*k*c);
                     -tau(i)/4/c -(m+c*tau(i))/(4*k*c) (m+c*tau(i))/(4*k*c*c)];
    det_Wcu_set_analytic(i) = (m+c*tau(i))/(8*k*c*c*tau(i))*p(i) - (m+c*tau(i))/(8*k*c) - tau(i)*tau(i)/8/c;
    iWcu_set_analytic(i,:,:) = (1/det_Wcu_set_analytic(i)) * iWcu_set_adj_analytic(i,:,:)*p(i);
end

for j = 1 : 3
    for i = 1 : length(tau)    
        dummy3(:,:) = Wcu_set_analytic(i,:,:);
        idummy3 = inv(dummy3);
        iWcu_diagonal_analytic(j,i) = idummy3(j,j);        
    end
end
% 
% figure;plot(f,1./p,'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('1/p')
% grid on
% 
% figure;plot(f,1./det_Wcu_set_analytic,'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('inverse of determinant of Wcu')
% grid on
% 
% figure;plot(f,iWcu_set_adj_analytic(:,1,1),'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('(1,1) of adjoint of Wcu')
% grid on
% 
% figure;plot(f,iWcu_set_adj_analytic(:,2,2),'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('(2,2) of adjoint of Wcu')
% grid on
% 
% figure;plot(f,iWcu_set_adj_analytic(:,3,3),'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('(3,3) of adjoint of Wcu')
% grid on


% figure;plot(f,iWcu_set_analytic(:,1,1),'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('(1,1) of inverse of Wcu')
% grid on
% 
% figure;plot(f,iWcu_set_analytic(:,2,2),'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('(2,2) of inverse of Wcu')
% grid on
% 
% figure;plot(f,iWcu_set_analytic(:,3,3),'b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('(3,3) of inverse of Wcu')
% grid on

%%
% qq = [1 30];
% 
% for z = 1 : length(qq)
%     q = qq(z);
% 
%     t_final = 10;
%     Ts = 0.01;
% 
%     xi = [0;0;0];
%     xf = [1;1;0];
% 
%     A = [An Bn;zeros(1,2) -1/tau(q)];
%     B = [zeros(2,1);1/tau(q)];
%     save A.mat A;
%     save B.mat B;
% 
%     ini = zeros(1,length(A)^2);
%     [t,y] = ode45(@gra_AB,[0 :Ts: t_final],ini);
%     for i = 1 : length(A)
%         for j = 1 : length(A)
%             Wc(i,j) = y(end,j+(i-1)*length(A));
%         end
%     end
%     rank_Wc = rank(Wc);
%     iWc = inv(Wc);
% 
%     u(z,:) = zeros(1,length(t));  
% 
%     for i = 1 : length(t)    
%         u(z,i) = B'*expm(A'*(t_final-t(i)))*iWc*(xf);
%     end
%     rho_from_input = sum(u(z,:).^2)*Ts
% 
% 
%     %response
%     sys = ss(A,B,eye(length(A)),0);
%     Y(z,:,:) = lsim(sys,u(z,:),t,xi);
% end
% 
% figure;plot(t,Y(1,:,1),'b','LineWidth',3)
% hold on
% plot(t,Y(2,:,1),'--r','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('time(sec)')
% title('1st state response')
% legend('f=1Hz','f=30Hz')
% grid on
% 
% figure;plot(t,Y(1,:,2),'b','LineWidth',3)
% hold on
% plot(t,Y(2,:,2),'--r','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('time(sec)')
% title('2nd state response')
% legend('f=1Hz','f=30Hz')
% grid on
% 
% figure;plot(t,Y(1,:,3),'k','LineWidth',3)
% hold on
% plot(t,Y(2,:,3),'r','LineWidth',3)
% plot(t,u(1,:),'--c','LineWidth',3)
% plot(t,u(2,:),'--b','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('time(sec)')
% title('3rd state response and input u')
% legend('3rd response w/ f=1Hz','3rd response w/ f=30Hz', 'input w/ f=1Hz', ' input w/ f=30Hz')
% grid on