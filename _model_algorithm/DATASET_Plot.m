%% data load
load Result_DOC_tf.mat;
load Result_DOC_fa.mat;
load Result_DOC_remaining_xf_openloop.mat;
load Result_DOC_remaining_xf_closedloop.mat;
load Result_DOC_rho.mat;                     

load Result_DODR_tf.mat;
load Result_DODR_fa.mat;
load Result_DODR_remaining_xf_openloop.mat;    %tf=30일때만
load Result_DODR_remaining_xf_closedloop.mat;  %tf=30일때만 
load Result_DODR_rho.mat;  

%% data plot - DOC
% fa에 따른 DOC
figure;plot(Result_DOC_fa,Result_DOC_rho(end,:),'b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
title('DOC at final time = 50')
grid on

%% data plot - DODR
% fa에 따른 DODR
figure;plot(Result_DODR_fa(1:3),Result_DODR_rho(end,1:3),'--b','LineWidth',3)
hold on
plot(Result_DODR_fa(3:end),Result_DODR_rho(end,3:end),'b','LineWidth',3)
set(gca,'Fontsize',14,'FontWeight','demi')
xlabel('cut off frequency of actuator(Hz)')
title('DODR at final time = 50')
grid on

% 
% %% final state energy calculation
% Stored_potential_energy_rho_set3_openloop = 0;Stored_kinetic_energy_rho_set3_openloop = 0;Stored_potential_energy_rho_set3 = 0;
% Stored_kinetic_energy_rho_set3 = 0;Stored_kinetic_energy_rho_set3_openloop_wo_velocity = 0;Stored_kinetic_energy_rho_set3_wo_velocity = 0;
% Mv = M; Mv(1,1) = 0;
% 
% for k = 1 : 20
%     xf1(:,1) = Result_DODR_remaining_xf_closedloop(:,k,:);  xf1_openloop(:,1) = Result_DODR_remaining_xf_openloop(:,k,:);
%     xf1(3) = 10+xf1(3);  xf1_openloop(3) = 10+xf1_openloop(3); 
% %     xi(3) = 0;  xi_openloop(3) = 0;     
%     
%     [Stored_potential_energy_rho_set3_openloop_mat,Stored_kinetic_energy_rho_set3_openloop_mat] = Stored_energy_by_disturbance(xi,xf1_openloop',M,K,Ts,v_initial);
%     [Stored_potential_energy_rho_set3_mat,Stored_kinetic_energy_rho_set3_mat] = Stored_energy_by_disturbance(xi,xf1',M,K,Ts,v_initial);
%     
%     [dummy,Stored_kinetic_energy_rho_set3_openloop_mat_wo_velocity] = Stored_energy_by_disturbance(xi,xf1_openloop',Mv,K,Ts,v_initial);
%     [dummy,Stored_kinetic_energy_rho_set3_mat_wo_velocity] = Stored_energy_by_disturbance(xi,xf1',Mv,K,Ts,v_initial);
%     
%     Stored_potential_energy_rho_set3_openloop = [Stored_potential_energy_rho_set3_openloop; Stored_potential_energy_rho_set3_openloop_mat];
%     Stored_kinetic_energy_rho_set3_openloop = [Stored_kinetic_energy_rho_set3_openloop; Stored_kinetic_energy_rho_set3_openloop_mat];
%     Stored_potential_energy_rho_set3 = [Stored_potential_energy_rho_set3; Stored_potential_energy_rho_set3_mat];
%     Stored_kinetic_energy_rho_set3 = [Stored_kinetic_energy_rho_set3; Stored_kinetic_energy_rho_set3_mat];
%     
%     Stored_kinetic_energy_rho_set3_openloop_wo_velocity = [Stored_kinetic_energy_rho_set3_openloop_wo_velocity; Stored_kinetic_energy_rho_set3_openloop_mat_wo_velocity];
%     Stored_kinetic_energy_rho_set3_wo_velocity = [Stored_kinetic_energy_rho_set3_wo_velocity; Stored_kinetic_energy_rho_set3_mat_wo_velocity];
% end
% 
% %% data plot - energy
% % Ep
% figure;plot(Result_DODR_fa,Stored_potential_energy_rho_set3_openloop(2:end),'b','LineWidth',3)
% hold on
% plot(Result_DODR_fa,Stored_potential_energy_rho_set3(2:end),'r','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% ylabel('energy[J]')
% title('stored potential energy')
% grid on
% legend('openloop','controlled')
% 
% % Ek
% figure;plot(Result_DODR_fa,Stored_kinetic_energy_rho_set3_openloop(2:end),'b','LineWidth',3)
% hold on
% plot(Result_DODR_fa,Stored_kinetic_energy_rho_set3(2:end),'r','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% ylabel('energy[J]')
% title('stored kinetic energy')
% grid on
% legend('openloop','controlled')
% 
% % Et
% figure;plot(Result_DODR_fa,Stored_potential_energy_rho_set3_openloop(2:end)+Stored_kinetic_energy_rho_set3_openloop(2:end),'b','LineWidth',3)
% hold on
% plot(Result_DODR_fa,Stored_potential_energy_rho_set3(2:end)+Stored_kinetic_energy_rho_set3(2:end),'r','LineWidth',3)
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% ylabel('energy[J]')
% title('stored total energy')
% grid on
% legend('openloop','controlled')
% 
% % rho*Ep
% figure;plot(Result_DODR_fa,Result_DODR_rho(end,:)'.*Stored_potential_energy_rho_set3(2:end),'b','LineWidth',3)
% hold on
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('multiplicationi of Ep and DODR')
% grid on
% 
% % rho*Ek
% figure;plot(Result_DODR_fa,Result_DODR_rho(end,:)'.*Stored_kinetic_energy_rho_set3(2:end),'b','LineWidth',3)
% hold on
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('multiplicationi of Ek and DODR')
% grid on
% 
% % rho*Et
% figure;plot(Result_DODR_fa,Result_DODR_rho(end,:)'.*(Stored_potential_energy_rho_set3(2:end)+Stored_kinetic_energy_rho_set3(2:end)),'b','LineWidth',3)
% hold on
% set(gca,'Fontsize',14,'FontWeight','demi')
% xlabel('cut off frequency of actuator(Hz)')
% title('multiplicationi of Et and DODR')
% grid on