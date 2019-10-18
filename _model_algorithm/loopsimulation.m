%% tf와 fa별로 data 수집
finite_time_of_DOC = 50;
cutoff_frequency_of_actuator = [0.5 : 0.5 : 10];
% cutoff_frequency_of_actuator = [10 : 1 : 20];

Qf1 = [K(2:3,2:3) zeros(2,3);zeros(3,2) M];
Qf = zeros(7,7);
Qf(1:5,1:5) = Qf1;

% a = 1;
% a = [0.01 0.1 1 10 100 1000 10000];
alpha = 10^18;
Qf = alpha * Qf;
% for i = 1 : length(a)
%     Qf = Qf(:);
%     Qf_list(i,:) = a(i) * Qf;
% end

for i = 1 : length(a)   
    for j = 1 : length(cutoff_frequency_of_actuator)    
        [w,Sw] = Road_noise(roadnoise,a,b,v_initial,Ts,finite_time_of_DOC);
        [A_actuator,B_actuator,D_actuator,xi_actuator,xf_actuator,Ar_actuator,Br_actuator,Dr_actuator,xir_actuator,xfr_actuator] = Include_actuator(actuator_dynamics_on,type,An,Bn,Dn,Arn,Brn,Drn,xin,xfn,xirn,xfrn,cutoff_frequency_of_actuator(j));
        A = A_actuator; B = B_actuator; D = D_actuator;     
        xi = xi_actuator; xf = xf_actuator;  
        save A.mat A;save B.mat B; save D.mat D;  
%         Ar = Ar_actuator; Br = Br_actuator; Dr = Dr_actuator;
%         xir = xir_actuator; xfr = xfr_actuator;
%         save Ar.mat Ar;save Br.mat Br; save Dr.mat Dr;        
%         [Wd,Wc,Wcr,iWc,iWcr,rho,rho_rigid,t,tr] = DOCandDODR(A,B,D,Ar,Br,Dr,finite_time_of_DOC(i),Ts,Sw,xi,xf,xir,xfr);
%         [rho_from_input,rho_rigid_from_input,u,ur,Y,Yr] = InputandResponse(iWc,iWcr,A,B,D,Ar,Br,Dr,t,tr,Ts,xi,xf,xir,xfr,type,w,finite_time_of_DOC);
%         [rho_new1,xf_remaining,rho_remaining,xf_openloop] = Cal_u_remaining(finite_time_of_DOC(i),Y,Yr,t,A,B,iWc,type,rho,rho_from_input,Ts,xf,v_initial);
        
%         [SR_cost] = StochasticRegulator(A,B,D,finite_time_of_DOC,Sw,xi,xf,Q,Qf_list(i,:));
        Y = 0;
        [Y_SR,cost_SR,P0_SR,P_list_SR,sum_us,final_SR] = StochasticRegulator(A,B,D,finite_time_of_DOC,Sw,xi,xf,Q,Qf,w,Y);
%         [Y_LV,u_LV,cost_LV,sum_u_LV] = TerminalControl1(A,B,D,finite_time_of_DOC,Sw,xi,w,Y);
        [i j]         
        SR_Set(i,j) = cost_SR;
%         LV_Set(i,j) = cost_LV;
%         LV_u_Set(i,j) = sum_u_LV;
    end
end
figure;plot(cutoff_frequency_of_actuator,SR_Set,'b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
% title('SR with alpha=10000')
grid on
set(gcf,'Position', [3090 50 560 420])
title('SR for modified DODR')

% figure;plot(SR_Set)
% figure;plot(LV_Set)
% figure;plot(LV_u_Set)

% save LV_Set.mat LV_Set;
%% 수집한 data 재정렬
% for i = 1 : length(finite_time_of_DOC)
%     for j = 1 : length(cutoff_frequency_of_actuator)
%         rho_set2_mat(i,j) = rho_set2((i-1)*length(cutoff_frequency_of_actuator) + j);
%     end
% end
%  
% 
% for i = 1 : length(finite_time_of_DOC)
%     for j = 1 : length(cutoff_frequency_of_actuator)
%         rho_set3_mat(i,j,:) = rho_set3_xf_remaining((i-1)*length(cutoff_frequency_of_actuator) + j,:);
%         rho_set3_openloop_mat(i,j,:) = rho_set3_xf_openloop((i-1)*length(cutoff_frequency_of_actuator) + j,:);
%     end
%  end
