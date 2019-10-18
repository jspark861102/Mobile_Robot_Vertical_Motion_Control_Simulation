% function [rho_new,xf_remaining,u_remaining] = Cal_u_remaining(finite_time_of_DOC,Y,t,A,B,iWc,type,rho,rho_from_input,Ts);
function [rho_new1,xf_remaining,rho_remaining,xf_openloop] = Cal_u_remaining(finite_time_of_DOC,Y,Yr,t,A,B,iWc,type,rho,rho_from_input,Ts,xf,v_initial);

%%%%%%%%%%%% calculate u_remaining %%%%%%%%%%%%%%%%
t_final = finite_time_of_DOC;

xf_remaining = Y(end,:)' - xf;
xf_remaining(6:7) = 0;
xf_remaining(3) = xf_remaining(3) - v_initial;

xf_remaining;

xf_openloop = Yr(end,:)' - xf;
xf_openloop(6:7) = 0;
xf_openloop(3) = xf_openloop(3) - v_initial;

xf_openloop;

rho_remaining = trace(iWc*(xf_remaining*xf_remaining'));

%1
rho_new1 = rho_remaining/rho;

%2
rho_new2 = (rho_remaining+rho)/rho;

%3
rho_new3 = (rho_remaining)*rho;

%4
rho_new4 = rho + rho_remaining;
%% from input - expectation이 문제이고..
% % find DOC for remianing state
% for i = 1 : length(t)    
%     u_remaining(:,i) = B'*expm(A'*(t_final-t(i)))*iWc*xf_remaining;
% end
% 
% if type/2 == 2
%     rho_remaining = (sum(u_remaining(1,:).^2)+sum(u_remaining(2,:).^2))*Ts;
%     rho_new = rho_remaining / rho_from_input;
% else
%     rho_remaining = sum(u_remaining(1,:).^2)*Ts;
%     rho_new = rho_remaining / rho_from_input;
% end
